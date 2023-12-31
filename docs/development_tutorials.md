# 开发

Carla 是开源的，并且具有高度可扩展性。这允许用户创建自定义功能或内容以满足专门应用程序或特定需求。以下教程详细介绍了如何使用 Carla 代码库实现特定的开发目标：

- [__进行发布__](tuto_D_make_release.md)
- [__升级内容__](tuto_D_contribute_assets.md)
- [__创建语义标签__](tuto_D_create_semantic_tags.md)
- [__创建新传感器__](tuto_D_create_sensor.md)
- [__性能基准测试__](adv_benchmarking.md)
- [__记录器文件格式__](ref_recorder_binary_file_format.md)
- [__碰撞边界__](tuto_D_generate_colliders.md)

## 进行发布

如果您想开发自己的 Carla 分支并发布代码版本，请遵循 [__本指南__](tuto_D_make_release.md)。

## 升级内容

我们的内容驻留在单独的 Git LFS 存储库中。作为构建系统的一部分，我们生成并上传一个包，其中包含带有当前日期和提交标记的该内容的最新版本。我们定期升级 Carla 存储库，并提供最新版本内容包的链接。请按照 [__这些说明__](tuto_D_contribute_assets.md) 升级内容。


## 创建语义标签

Carla 有一组已定义的语义标签，适合大多数用例。但是，如果您需要其他课程，您可以按照 [__本指南__](tuto_D_create_semantic_tags.md) 中的详细说明添加它们。

## 创建新传感器

您可以修改 Carla 的 C++ 代码来为您的自定义用例创建新的传感器。请在 [__此处__](tuto_D_create_sensor.md) 查找详细信息。

## 性能基准测试

Carla 有一个基准测试脚本，可帮助对系统性能进行基准测试。在 [__这里__](adv_benchmarking.md) 找到完整的详细信息。

## 记录器文件格式

有关记录器二进制文件格式的详细信息，请参阅 [__此处__](ref_recorder_binary_file_format.md) 。

## 生成碰撞边界

有关为车辆生成更准确的碰撞边界的详细信息，请参阅 [__本指南__](tuto_D_generate_colliders.md) 。
