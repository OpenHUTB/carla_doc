# RLlib 集成

RLlib 集成带来了 [Ray/RLlib](https://github.com/ray-project/ray) 库和 Carla 之间的支持，允许轻松使用 Carla 环境进行训练和推理。Ray 是一个开源框架，为构建分布式应用程序提供了简单、通用的 API。Ray 与 RLlib（一个可扩展的强化学习库）和 Tune（一个可扩展的超参数调整库）打包在一起。

RLlib 集成允许用户创建和使用 Carla 作为 Ray 的环境，并将该环境用于训练和推理目的。该集成可以在本地使用，也可以通过 AWS 在云中使用。

在本指南中，我们将概述在本地和 AWS 上运行 RLlib 集成所需的要求、集成存储库的结构、如何使用该库的概述，以及如何使用 Carla 设置 Ray 实验的示例一个环境。

- [__在你开始之前__](#before_you_begin)
    - [本地运行的要求](#requirements_for_running_locally)
    - [在 AWS 云上运行的要求](#requirements_for_running_on_aws_cloud)
- [__RLlib 仓库结构__](#rllib_repository_structure)
- [__创建您自己的实验__](#creating_your_own_experiment)
    - [实验类](#1_the_experiment_class)
    - [环境配置](#2_the_environment_configuration)
    - [训练和推理脚本](#3_the_training_and_inference_scripts)
- [__DQN 示例__](#dqn_example)
- [__在 AWS 上运行__](#running_on_aws)
    - [配置 AWS](#configure_aws)
    - [创建训练 AMI](#create_the_training_ami)
    - [配置集群](#configure_the_cluster)
    - [运行训练](#run_the_training)
    - [在 AWS 上运行 DQN 示例](#running_the_dqn_example_on_aws)

---

## 在你开始之前 <span id="before_you_begin"></span>

- 从 [GitHub](https://github.com/carla-simulator/rllib-integration/tree/main) 下载 RLlib 集成或直接克隆仓库：

```sh
git clone https://github.com/carla-simulator/rllib-integration.git
```

- 要求会有所不同，具体取决于您是在本地运行还是在 AWS 上运行：

>###### 本地运行的要求 <span id="requirements_for_running_locally"></span>

>>- [安装 Carla 的软件包版本](https://github.com/carla-simulator/carla/releases) 并导入 [附加资源](https://carla.readthedocs.io/en/latest/start_quickstart/#import-additional-assets) 。__推荐的版本是 Carla 0.9.11__ ，因为集成是使用此版本设计和测试的。其他版本可能兼容，但尚未经过充分测试，因此请自行决定使用这些版本。
>>- 导航到 RLlib 集成存储库的根文件夹并安装 Python 要求：

    pip3 install -r requirements.txt

>>- 通过运行以下命令设置环境变量以查找 Carla 包或添加`CARLA_ROOT=path/to/carla`到您的`.bashrc`文件中：

    export CARLA_ROOT=path/to/carla

>###### 在 AWS 云上运行的要求 <span id="requirements_for_running_on_aws_cloud"></span>

>>- 在 RLlib 集成存储库中找到的安装脚本中会自动满足在 AWS 上运行的要求。[在“在 AWS 上运行”](#running-on-aws) 部分中查找更多详细信息。

---

## RLlib 存储库结构 <span id="rllib_repository_structure"></span>

存储库分为三个目录：

- `rllib_integration` 包含与 Carla 相关的所有基础设施以及如何设置 Carla 服务器、客户端和参与者。这提供了所有训练和测试实验必须遵循的基本结构。
- `aws` 具有在 AWS 实例中运行所需的文件。`aws_helper.py`提供了多种可简化 EC2 实例管理的功能，包括实例创建以及发送和接收数据。
- 根目录中的 `dqn_example` 和 `dqn_*` 文件提供了一个易于理解的示例，说明如何使用 Carla 作为环境来设置 Ray 实验。 

---

## 创建您自己的实验 <span id="creating_your_own_experiment"></span>

本节概述了如何创建您自己的实验。更具体的示例请参见下一节[“DQN 示例”](#dqn-example) 。

您将需要创建至少四个文件：

- 实验类
- 环境配置
- 训练和推理脚本


#### 1. 实验类 <span id="1_the_experiment_class"></span>

要使用 Carla 环境，您需要定义训练实验。Ray 要求环境返回一系列特定信息。您可以在 [`rllib-integration/rllib_integration/carla_env.py`][carlaEnv] 中查看有关 Carla 环境的详细信息。

Ray 所需的信息取决于您的具体实验，因此所有实验都应继承自[`BaseExperiment`][baseExperiment]。此类包含您自己的实验需要覆盖的所有函数。这些都是与训练的行动、观察和奖励相关的功能。

[carlaEnv]: https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/carla_env.py
[baseExperiment]: https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/base_experiment.py#L41

#### 2. 环境配置 <span id="2_the_environment_configuration"></span>

实验应该通过`.yaml`文件进行配置。通过配置文件传递的任何设置都将覆盖默认设置。下面解释了不同默认设置的位置。

配置文件有三个主要用途：

1. 设置大部分 Carla 服务器和客户端设置，例如超时或地图质量。请参阅 [此处][defaultCarlaSettings] 的默认值。 
2. 设置特定于您的实验的变量，以及指定城镇条件以及自我车辆及其传感器的生成。默认设置可在 [此处][defaultExperimentSettings] 找到，并提供如何设置传感器的示例。
3. 配置特定于 [Ray 的训练][raySettings] 设置。这些设置与所使用的特定训练器相关。如果您使用的是内置模型，则可以在此处应用其设置。 

[defaultCarlaSettings]: https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/carla_core.py#L23
[defaultExperimentSettings]: https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/base_experiment.py#L12
[raySettings]: https://github.com/ray-project/ray/blob/master/rllib/agents/trainer.py

#### 3. 训练和推理脚本 <span id="3_the_training_and_inference_scripts"></span>

最后一步是创建您自己的训练和推理脚本。这部分完全由您决定，并且依赖于 Ray API。如果您想创建自己的特定模型，请查看 [Ray 的自定义模型文档][rayCustomModel] 。

[rayCustomModel]: https://docs.ray.io/en/master/rllib-models.html#custom-models-implementing-your-own-forward-logic

---

## DQN 示例 <span id="dqn_example"></span>

本节以上一节为基础，展示如何使用 [BirdView pseudosensor][birdview] 伪传感器和 Ray 的 [DQNTrainer][dqntrainer] 进行 RLlib 集成的具体示例。

[birdview]: https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/sensors/bird_view_manager.py
[dqntrainer]: https://github.com/ray-project/ray/blob/master/rllib/agents/dqn/dqn.py#L285

DQN示例的结构如下：

- __实验类__: [`DQNExperiment`][dqnExperiment]，覆盖 `BaseExperiment` 类的方法。
- __配置文件__: [`dqn_example/dqn_config.yaml`][dqnConfig]
- __训练文件__: [`dqn_train.py`][dqnTrain]
- __推理文件__:
    - __有 Ray__: [`dqn_inference_ray.py`][dqnInferenceRay] 
    - __没有 Ray__: [`dqn_inference.py`][dqnInference]

[dqnExperiment]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_example/dqn_experiment.py#L19
[dqnConfig]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_example/dqn_config.yaml
[dqnTrain]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_train.py
[dqnInferenceRay]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_inference_ray.py
[dqnInference]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_inference.py

要在本地运行该示例：

1. 安装 pytorch:

        pip3 install -r dqn_example/dqn_requirements.txt

2. 运行训练文件：

        python3 dqn_train.py dqn_example/dqn_config.yaml --name dqn        

!!! 笔记
    默认配置使用 1 个 GPU 和 12 个 CPU，因此如果您的本地计算机没有该容量，请减小 [配置文件][dqnConfig] 中的数字。
    
    如果遇到内存不足问题，请考虑减少该`buffer_size`参数。

---

## 在 AWS 上运行 <span id="running_on_aws"></span>

本节介绍如何使用 RLlib 集成在 AWS EC2 实例上自动运行训练和推理。为了处理实例的缩放，我们使用[Ray autoscaler API][rayAutoscaler]。

[rayAutoscaler]: https://docs.ray.io/en/latest/cluster/index.html

#### 配置 AWS <span id="configure_aws"></span>

您需要正确配置 boto3 环境。请点击 [此处][awsBoto3] 了解更多信息。

[awsBoto3]: https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html

#### 创建训练 AMI <span id="create_the_training_ami"></span>

使用提供的 [`aws_helper.py`][awsHelper] 脚本通过运行以下命令自动创建训练所需的映像，并传入基础映像的名称和 `install.sh` 在以下位置  [`rllib-integration/aws/install`][installsh] 找到的安装脚本：

    python3 aws_helper.py create-image --name <AMI-name> --installation-scripts <installation-scripts> --instance-type <instance-type> --volume-size <volume-size>

[awsHelper]: https://github.com/carla-simulator/rllib-integration/blob/main/aws/aws_helper.py
[installsh]: https://github.com/carla-simulator/rllib-integration/blob/main/aws/install/install.sh

#### 配置集群 <span id="configure_the_cluster"></span>

创建图像后，将会输出带有图像信息的输出。要使用 Ray 自动缩放器，请使用输出中的信息更新 [autoscaler configuration file][autoscalerSettings] 中的`<ImageId>` 和 `<SecurityGroupIds>` 设置。


[autoscalerSettings]: https://docs.ray.io/en/latest/cluster/config.html

#### 运行训练 <span id="run_the_training"></span>

创建镜像后，您可以使用 Ray 的 API 在集群上运行训练：

1. 初始化集群：

        ray up <autoscaler_configuration_file>

2. （可选）如果集群初始化后本地代码被修改，请执行以下命令更新：

        ray rsync-up <autoscaler_configuration_file> <path_to_local_folder> <path_to_remote_folder>

3. 运行训练：

        ray submit <autoscaler_configuration_file> <training_file>

4. （可选）监控集群状态：

        ray attach <autoscaler_configuration_file>
        watch -n 1 ray status

5. 关闭集群：

        ray down <autoscaler_configuration_file>


#### 在 AWS 上运行 DQN 示例 <span id="running_the_dqn_example_on_aws"></span>

要在 AWS 上运行 DQN 示例：

1. 通过将 [`dqn_example/dqn_autoscaler.yaml`][dqnAutoscaler] 配置传递给以下命令来创建映像：

        python3 aws_helper.py create-image --name <AMI-name> --installation-scripts install/install.sh --instance-type <instance-type> --volume-size <volume-size>

[dqnAutoscaler]: https://github.com/carla-simulator/rllib-integration/blob/main/dqn_example/dqn_autoscaler.yaml

2. 使用上一个命令 [`dqn_autoscaler.yaml`][dqnAutoscaler] 提供的信息更新`<ImageId>`和`<SecurityGroupIds>`设置。

3. 初始化集群：

        ray up dqn_example/dqn_autoscaler.yaml

4. （可选）使用本地更改更新远程文件：

        ray rsync-up dqn_example/dqn_autoscaler.yaml dqn_example .
        ray rsync-up dqn_example/dqn_autoscaler.yaml rllib_integration .

5. 运行训练：

        ray submit dqn_example/dqn_autoscaler.yaml dqn_train.py -- dqn_example/dqn_config.yaml --auto

6.（可选）监控集群状态：

        ray attach dqn_example/dqn_autoscaler.yaml
        watch -n 1 ray status

7. 关闭集群：

        ray down dqn_example/dqn_autoscaler.yaml

---

本指南概述了如何在 AWS 和本地计算机上安装和运行 RLlib 集成。如果您对本指南有任何疑问或遇到任何问题，请随时在 [论坛](https://github.com/carla-simulator/carla/discussions/) 中发帖或在 [GitHub](https://github.com/carla-simulator/rllib-integration) 上提出问题。

