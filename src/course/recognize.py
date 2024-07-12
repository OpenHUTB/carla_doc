import torch
from torchvision import transforms
from PIL import Image
import torchreid


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


# 数据管理器
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

# 模型
model = torchreid.models.build_model(
    name="resnet50",
    num_classes=datamanager.num_train_pids,
    loss="softmax",
    pretrained=True  # 这里已经加载了 ImageNet 预训练权重
)

# 将模型移动到GPU
model = model.cuda()
model.eval()


def infer(model, image_tensor):
    with torch.no_grad():
        image_tensor = image_tensor.cuda()
        features = model(image_tensor)
    return features


def cosine_similarity(tensor1, tensor2):
    cos = torch.nn.CosineSimilarity(dim=1, eps=1e-6)
    return cos(tensor1, tensor2).item()


image_path1 = 'reid-data/market1501/bounding_box_test/0000_c1s1_000151_01.jpg'
image_path2 = 'reid-data/market1501/bounding_box_test/0000_c1s1_000151_01.jpg'

image1 = preprocess_image(image_path1)
image2 = preprocess_image(image_path2)

features1 = infer(model, image1)
features2 = infer(model, image2)

similarity = cosine_similarity(features1, features2)

print(f"Cosine similarity between the two images: {similarity}")
