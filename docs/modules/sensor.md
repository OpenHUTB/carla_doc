import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from scipy.interpolate import RegularGridInterpolator
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm

# ============ 配置 ============ #
PRINT_PATHS = True
DEM_PATH = "DEM_Hunan_0.1deg.npy"
TERRAIN_LABEL_PATH = "terrain_label_0.1deg.npy"
month_days = [f"2025-01-{str(i).zfill(2)}" for i in range(1, 32)]

parser = argparse.ArgumentParser()
parser.add_argument("--gpu", type=int, default=2)
args = parser.parse_args()

device = torch.device(f"cuda:{args.gpu}" if torch.cuda.is_available() else "cpu")

def log_print(msg, file="training_log_phys.txt"):
    print(msg)
    with open(file, "a", encoding="utf-8") as f:
        f.write(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] {msg}\n")

# ============ 模型定义 ============ #
class TinyFormer(nn.Module):
    def __init__(self, input_dim=18, embed_dim=64, num_heads=4, mlp_dim=128):
        super().__init__()
        self.input_proj = nn.Linear(input_dim, embed_dim)
        self.self_attn = nn.MultiheadAttention(embed_dim, num_heads, batch_first=True)
        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, mlp_dim),
            nn.ReLU(),
            nn.Linear(mlp_dim, embed_dim)
        )
        self.norm1 = nn.LayerNorm(embed_dim)
        self.norm2 = nn.LayerNorm(embed_dim)
        self.output_layer = nn.Linear(embed_dim, 3)

    def forward(self, x):
        # x 的形状为 [B, input_dim]
        x = self.input_proj(x).unsqueeze(1)  # [B, 1, embed_dim]
        attn_output, _ = self.self_attn(x, x, x)
        x = self.norm1(x + attn_output)
        x = self.norm2(x + self.mlp(x))
        return self.output_layer(x.squeeze(1))  # [B, 3]

# ============ Dataset 定义 ============ #
class GridSampleDataset(Dataset):
    def __init__(self, samples):
        self.samples = samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        x, y = self.samples[idx]
        return torch.tensor(x, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)

# ============ 数据生成 ============ #
def generate_samples(base_025, base_01, date, hours=240):
    cache_dir = "cache_phys"
    os.makedirs(cache_dir, exist_ok=True)
    cache_path = os.path.join(cache_dir, f"samples_{date}.npy")
    if os.path.exists(cache_path):
        if PRINT_PATHS:
            log_print(f"🗃 加载缓存样本: {cache_path}")
        return list(np.load(cache_path, allow_pickle=True))

    results = []
    lat_025 = np.linspace(90, -90, 721)
    lon_025 = np.linspace(0, 359.75, 1440)
    lat_01 = np.linspace(90, -90, 1801)
    lon_01 = np.linspace(0, 359.9, 3600)
    lat_min, lat_max = 24.5, 30.5
    lon_min, lon_max = 108.5, 114.5
    lat_idx = np.where((lat_01 >= lat_min) & (lat_01 <= lat_max))[0]
    lon_idx = np.where((lon_01 >= lon_min) & (lon_01 <= lon_max))[0]
    lat_hunan, lon_hunan = lat_01[lat_idx], lon_01[lon_idx]
    lon_grid, lat_grid = np.meshgrid(lon_hunan, lat_hunan)
    points = np.stack([lat_grid.ravel(), lon_grid.ravel()], axis=-1)

    dem_crop = np.load(DEM_PATH)[:len(lat_hunan), :len(lon_hunan)]
    terrain_crop = np.load(TERRAIN_LABEL_PATH)[:len(lat_hunan), :len(lon_hunan)]

    prev_path = os.path.join(base_025, date, "23-surface.npy")

    for h in range(hours):
        predict_dt = np.datetime64(date) + np.timedelta64(1, "D") + np.timedelta64(h, "h")
        pred_stamp = str(predict_dt).replace("T", "-")[:13] + "-00"
        predict_path = os.path.join(base_025, date, "inference_data", str(predict_dt)[:10], f"output_surface_{pred_stamp}.npy")
        target_path = os.path.join(base_01, str(predict_dt)[:10], pred_stamp, f"input_surface_{pred_stamp}.npy")

        if PRINT_PATHS:
            log_print(f"[路径检查] hour {h} @ {date}")
            log_print(f"  ➤ prev:   {prev_path}")
            log_print(f"  ➤ pred:   {predict_path}")
            log_print(f"  ➤ target: {target_path}")

        if not all(os.path.exists(p) for p in [prev_path, predict_path, target_path]):
            continue

        prev = np.load(prev_path)
        pred = np.load(predict_path)
        target = np.load(target_path)

        def interp(data):
            return np.stack([
                RegularGridInterpolator((lat_025, lon_025), data[i], bounds_error=False, fill_value=np.nan)(points).reshape(lat_grid.shape)
                for i in range(4)
            ], axis=0)

        prev_interp, pred_interp = interp(prev), interp(pred)
        target_crop = target[:, lat_idx[0]:lat_idx[-1]+1, lon_idx[0]:lon_idx[-1]+1]

        for i in range(lat_grid.shape[0]):
            for j in range(lat_grid.shape[1]):
                if np.any(np.isnan(target_crop[[1, 2, 0], i, j])) or \
                   np.any(np.isnan(prev_interp[[1, 2, 3], i, j])) or \
                   np.any(np.isnan(pred_interp[[1, 2, 3], i, j])) or \
                   np.isnan(dem_crop[i, j]):
                    continue

                hour = int(str(predict_dt)[11:13])
                cos_hour = np.cos(2 * np.pi * hour / 24)
                sin_hour = np.sin(2 * np.pi * hour / 24)
                hour_norm = hour / 24
                is_daytime = 1.0 if 7 <= hour <= 19 else 0.0

                u_prev = prev_interp[1, i, j]
                v_prev = prev_interp[2, i, j]
                u_curr = pred_interp[1, i, j]
                v_curr = pred_interp[2, i, j]
                temp_prev = prev_interp[3, i, j] - 273.15
                temp_curr = pred_interp[3, i, j] - 273.15
                temp_diff = temp_curr - temp_prev
                wind_dir_diff = np.sin(np.arctan2(v_curr, u_curr) - np.arctan2(v_prev, u_prev))
                vmag_prev = np.sqrt(u_prev**2 + v_prev**2)
                vmag_curr = np.sqrt(u_curr**2 + v_curr**2)
                kinetic_diff = 0.5 * (vmag_curr**2 - vmag_prev**2)
                lapse_rate = temp_diff / (dem_crop[i, j] + 1e-5)
                thermal_flux = temp_diff * vmag_curr

                # 构造物理增强后的特征向量（长度18）
                x = np.array([
                    u_prev, v_prev, temp_prev,
                    u_curr, v_curr, temp_curr,
                    dem_crop[i, j], h / hours,
                    cos_hour, sin_hour, hour_norm, is_daytime,
                    terrain_crop[i, j], wind_dir_diff, temp_diff,
                    kinetic_diff, lapse_rate, thermal_flux
                ], dtype=np.float32)

                y = np.array([
                    target_crop[1, i, j],
                    target_crop[2, i, j],
                    target_crop[0, i, j] - 273.15
                ], dtype=np.float32)

                results.append((x, y))

        # 更新prev_path为当前预测数据的路径
        prev_path = predict_path

    np.save(cache_path, np.array(results, dtype=object))
    return results

# ============ 训练函数 ============ #
def train_model(train_loader, val_loader, tag="2c", max_epochs=100, patience=5, lr=1e-3):
    model = TinyFormer(input_dim=18).to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max_epochs)
    best_loss = float("inf")
    no_improve = 0
    train_losses, val_losses = [], []
    os.makedirs("models", exist_ok=True)

    mse_loss_fn = nn.MSELoss()
    l1_loss_fn = nn.L1Loss()
    eps = 1e-5

    for epoch in range(1, max_epochs + 1):
        model.train()
        total_loss = 0
        for x, y in tqdm(train_loader, desc=f"🟢 Training Epoch {epoch}"):
            x, y = x.to(device), y.to(device)
            pred = model(x)

            # 基础损失项
            loss_mse = mse_loss_fn(pred, y)
            loss_l1 = l1_loss_fn(pred, y)

            # 正则项 1：动能守恒
            ek_pred = 0.5 * (pred[:, 0]**2 + pred[:, 1]**2)
            ek_true = 0.5 * (y[:, 0]**2 + y[:, 1]**2)
            loss_kinetic = mse_loss_fn(ek_pred, ek_true)

            # 正则项 2：温度递减率
            predicted_lapse = (pred[:, 2] - x[:, 2]) / (x[:, 6] + eps)
            target_lapse = x[:, 16]
            loss_lapse = mse_loss_fn(predicted_lapse, target_lapse)

            # 正则项 3：热通量一致性
            vmag_pred = torch.sqrt(pred[:, 0]**2 + pred[:, 1]**2)
            flux_pred = (pred[:, 2] - x[:, 2]) * vmag_pred
            flux_true = x[:, 17]
            loss_flux = mse_loss_fn(flux_pred, flux_true)

            # 总损失
            loss = 0.7 * loss_mse + 0.3 * loss_l1 + 0.1 * loss_kinetic + 0.1 * loss_lapse + 0.05 * loss_flux

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        scheduler.step()

        # 验证阶段
        model.eval()
        val_loss_sum = 0
        with torch.no_grad():
            for x, y in tqdm(val_loader, desc="🔵 Validating", leave=False):
                x, y = x.to(device), y.to(device)
                pred = model(x)

                loss_mse = mse_loss_fn(pred, y)
                loss_l1 = l1_loss_fn(pred, y)
                ek_pred = 0.5 * (pred[:, 0]**2 + pred[:, 1]**2)
                ek_true = 0.5 * (y[:, 0]**2 + y[:, 1]**2)
                loss_kinetic = mse_loss_fn(ek_pred, ek_true)
                predicted_lapse = (pred[:, 2] - x[:, 2]) / (x[:, 6] + eps)
                target_lapse = x[:, 16]
                loss_lapse = mse_loss_fn(predicted_lapse, target_lapse)
                vmag_pred = torch.sqrt(pred[:, 0]**2 + pred[:, 1]**2)
                flux_pred = (pred[:, 2] - x[:, 2]) * vmag_pred
                flux_true = x[:, 17]
                loss_flux = mse_loss_fn(flux_pred, flux_true)

                loss = 0.7 * loss_mse + 0.3 * loss_l1 + 0.1 * loss_kinetic + 0.1 * loss_lapse + 0.05 * loss_flux
                val_loss_sum += loss.item()

        avg_train = total_loss / len(train_loader)
        avg_val = val_loss_sum / len(val_loader)
        train_losses.append(avg_train)
        val_losses.append(avg_val)
        log_print(f"Epoch {epoch}: Train={avg_train:.4f}, Val={avg_val:.4f}")

        if avg_val < best_loss:
            best_loss = avg_val
            no_improve = 0
            torch.save(model.state_dict(), f"models/best_model_2c.pth")
            log_print(f"✅ 保存最优模型至: models/best_model_2c.pth")
        else:
            no_improve += 1
            if no_improve >= patience:
                log_print("⛔ 达到早停标准，停止训练")
                break

    plt.figure()
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Val Loss')
    plt.legend()
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training and Validation Loss')
    plt.grid()
    plt.savefig("loss_curve_phys_2c.png")
    plt.close()

    torch.save(model.state_dict(), "models/final_model_2c.pth")
    log_print("📦 最终模型保存至: models/final_model_2c.pth")
    return model


def main():
    base_025, base_01 = "0.25win", "0.1wind"
    samples = []
    for date in month_days:
        samples.extend(generate_samples(base_025, base_01, date))
    np.random.shuffle(samples)
    n = len(samples)
    # 根据数据量划分训练集与验证集（80%-10%）
    train_loader = DataLoader(
        GridSampleDataset(samples[:int(n * 0.8)]),
        batch_size=4096,
        shuffle=True,
        num_workers=8,
        pin_memory=True
    )
    val_loader = DataLoader(
        GridSampleDataset(samples[int(n * 0.8):int(n * 0.9)]),
        batch_size=4096,
        num_workers=8,
        pin_memory=True
    )

    train_model(train_loader, val_loader, tag="2c")

if __name__ == "__main__":
    main()
