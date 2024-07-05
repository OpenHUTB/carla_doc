import os
import pandas as pd
import numpy as np
from scipy import stats

# 定义结果目录名
results_directory = 'Results'
num_vehicles = 2  # 假设有2辆车，可以根据实际车辆数量进行调整

# 读取多车辆的跟踪误差数据
error_column_names = ['横向误差（m）', '航向误差（rad）']

# 存储每辆车的误差数据
all_cte = []
all_he = []

for vehicle_id in range(num_vehicles):
    error_file = os.path.join(results_directory, f'Tracking Error Log_{vehicle_id}.csv')
    if os.path.exists(error_file):
        error_data = pd.read_csv(error_file, names=error_column_names)
        print(f"车辆 {vehicle_id} 的跟踪误差数据（前5条）：")
        print(error_data.head())

        # 加载数据
        cte, he = error_data.iloc[:, 0].values, error_data.iloc[:, 1].values
        all_cte.append(cte)
        all_he.append(he)
    else:
        print(f"车辆 {vehicle_id} 的文件 {error_file} 不存在")

# 计算每辆车的横向误差指标
for vehicle_id, cte in enumerate(all_cte):
    cte_mae = np.mean(np.abs(cte))
    cte_mse = np.mean(np.square(cte))
    cte_rmse = np.sqrt(cte_mse)
    print(f"车辆 {vehicle_id} 的横向误差:")
    print(f"MAE:  {cte_mae:.6f} m")
    print(f"MSE:  {cte_mse:.6f} m")
    print(f"RMSE: {cte_rmse:.6f} m")

# 计算每辆车的航向误差指标
for vehicle_id, he in enumerate(all_he):
    he_mae = np.mean(np.abs(he))
    he_mse = np.mean(np.square(he))
    he_rmse = np.sqrt(he_mse)
    print(f"车辆 {vehicle_id} 的航向误差:")
    print(f"MAE:  {he_mae:.6f} rad")
    print(f"MSE:  {he_mse:.6f} rad")
    print(f"RMSE: {he_rmse:.6f} rad")

# 读取多车辆的延迟数据
latency_column_names = ['延迟（ms）']

# 存储每辆车的延迟数据
all_latency = []

for vehicle_id in range(num_vehicles):
    latency_file = os.path.join(results_directory, f'Latency Log_{vehicle_id}.csv')
    if os.path.exists(latency_file):
        latency_data = pd.read_csv(latency_file, names=latency_column_names)
        print(f"车辆 {vehicle_id} 的延迟数据（前5条）：")
        print(latency_data.head())

        # 加载延迟数据
        latency = latency_data.iloc[:, 0].values
        latency = latency[latency != 0]  # 排除为0的延迟值
        all_latency.append(latency)
    else:
        print(f"车辆 {vehicle_id} 的文件 {latency_file} 不存在")

# 计算每辆车的延迟指标
for vehicle_id, latency in enumerate(all_latency):
    latency_mean = np.mean(latency)
    latency_median = np.median(latency)
    latency_mode = stats.mode(latency)
    print(f"车辆 {vehicle_id} 的延迟:")
    print(f"平均值: {latency_mean:.6f} ms")
    print(f"中位数: {latency_median:.6f} ms")
    print(f"众数:   {latency_mode.mode[0]:.6f} ms [出现次数 = {latency_mode.count[0]}]")

# 汇总所有车辆的延迟指标
if all_latency:
    combined_latency = np.concatenate(all_latency)
    combined_latency_mean = np.mean(combined_latency)
    combined_latency_median = np.median(combined_latency)
    combined_latency_mode = stats.mode(combined_latency)
    print("所有车辆的汇总延迟:")
    print(f"平均值: {combined_latency_mean:.6f} ms")
    print(f"中位数: {combined_latency_median:.6f} ms")
    print(f"众数:   {combined_latency_mode.mode[0]:.6f} ms [出现次数 = {combined_latency_mode.count[0]}]")