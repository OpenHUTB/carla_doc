# 行人再识别

该[脚本](../../src/course/recognize.py)的目标是通过使用预训练的ResNet-50模型，比较两张图片的特征相似性。主要步骤包括数据预处理、特征提取和计算相似性度量。本文档将详细说明每个步骤的实现。

### 环境配置

#### 依赖库

- `torch`: PyTorch库，用于深度学习和张量计算。
- `torchvision`: PyTorch的图像处理工具包，提供了常用的数据增强方法。
- `PIL` (`Pillow`): Python Imaging Library，用于图像处理。
- `torchreid`: 用于行人重识别的工具包。

可以通过以下命令安装所需库（安装时间较长）：

```
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### 代码结构

#### **1.数据预处理**

首先定义一个函数 `preprocess_image` 来预处理图像：

```
def preprocess_image(image_path):
    transform = transforms.Compose([
        transforms.Resize((256, 128)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    image = Image.open(image_path).convert('RGB')
    image = transform(image)
    image = image.unsqueeze(0)  # 增加批次维度
    return image
```

#### **2.数据管理器**

创建一个数据管理器，用于加载[Market1501数据集](https://www.kaggle.com/datasets/pengcw1/market-1501/data)：

```
datamanager = torchreid.data.ImageDataManager(
    root="reid-data",
    sources="market1501",
    targets="market1501",
    height=256,
    width=128,
    batch_size_train=32,
    batch_size_test=100,
    transforms=["random_flip", "random_crop"]
)
```

#### **3.模型构建与加载**

构建并加载预训练的ResNet-50模型：

```
model = torchreid.models.build_model(
    name="resnet50",
    num_classes=datamanager.num_train_pids,
    loss="softmax",
    pretrained=True  # 加载 ImageNet 预训练权重
)

# 将模型移动到GPU
model = model.cuda()
model.eval()
```

#### **4.特征提取与相似性计算**

定义函数 `infer` 和 `cosine_similarity` 来提取特征和计算余弦相似性：

```
def infer(model, image_tensor):
    with torch.no_grad():
        image_tensor = image_tensor.cuda()
        features = model(image_tensor)
    return features

def cosine_similarity(tensor1, tensor2):
    cos = torch.nn.CosineSimilarity(dim=1, eps=1e-6)
    return cos(tensor1, tensor2).item()
```

### 运行步骤

1. 确保安装所有依赖库。
2. 下载并准备好Market1501数据集。
3. 运行上述Python代码，确保图像路径正确。

### 注意事项

1. 确保你的显卡与当前安装的PyTorch版本兼容。
2. 测试数据集的文件夹目录正确，并且和脚本在同一级。如下所示：

```
reid-data/          
├── market1501/
    ├── bounding_box_train/
    ├── bounding_box_test/
    ├── query/
    └── ...
```

### 结果分析

代码运行后会输出两张图像的余弦相似性。相似性值接近1表示两张图像特征相似度高，接近0表示特征相似度低。