# osc2-runner

我们使用OpenScenario 2.0作为场景描述语言规范，设计并实现了相应的编译系统，可以自动将用OpenScenario 2.0描述的测试场景转换为基于scenario runner的测试场景，从而使用carla进行自动驾驶测试。

## Installation

**1. 安装JDK**

```
sudo apt install openjdk-17-jdk
```    
确认安装：
```
$ java -version
```
输出：
```
openjdk version "17.0.5" 2022-10-18
OpenJDK Runtime Environment (build 17.0.5+8-Ubuntu-2ubuntu120.04)
OpenJDK 64-Bit Server VM (build 17.0.5+8-Ubuntu-2ubuntu120.04, mixed mode, sharing)
```

**3、安装 Antlr 4.10.1**

```
sudo apt install curl
curl -O https://www.antlr.org/download/antlr-4.10.1-complete.jar
```

将.jar文件放入local/lib中
```
$ sudo cp antlr-4.10.1-complete.jar /usr/local/lib/
```

以下三个步骤用于配置环境变量并创建别名，以便可以轻松地从命令行使用antlr4。
```
$ sudo gedit ~/.bashrc
```
在末尾添加以下内容：
```
export CLASSPATH=".:/usr/local/lib/antlr-4.10.1-complete.jar:$CLASSPATH"
alias antlr4='java -jar /usr/local/lib/antlr-4.10.1-complete.jar'
alias grun='java org.antlr.v4.gui.TestRig'
```
进而：
```
source ~/.bashrc
```

在终端输入antlr4进行验证：
```
$ antlr4
ANTLR Parser Generator  Version 4.10.1
 -o ___              specify output directory where all output is generated
 -lib ___            specify location of grammars, tokens files
 -atn                generate rule augmented transition network diagrams
 -encoding ___       specify grammar file encoding; e.g., euc-jp
 -message-format ___ specify output style for messages in antlr, gnu, vs2005
 -long-messages      show exception details when available for errors and warnings
 -listener           generate parse tree listener (default)
 -no-listener        don't generate parse tree listener
 -visitor            generate parse tree visitor
 -no-visitor         don't generate parse tree visitor (default)
 -package ___        specify a package/namespace for the generated code
 -depend             generate file dependencies
 -D<option>=value    set/override a grammar-level option
 -Werror             treat warnings as errors
 -XdbgST             launch StringTemplate visualizer on generated code
 -XdbgSTWait         wait for STViz to close before continuing
 -Xforce-atn         use the ATN simulator for all predictions
 -Xlog               dump lots of logging info to antlr-timestamp.log
 -Xexact-output-dir  all output goes into -o dir regardless of paths/package
```

**4、安装antlr4运行时**
```
pip install antlr4-python3-runtime==4.10
```

**5、安装python依赖**
```
pip install -r requirements.txt
```

**6、安装 graphviz**

```
sudo apt-get install graphviz
```


**7、配置 carla**
    
(1) 下载 [carla版本](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz)


(2) 将carla安装包解压到某个目录。

在Ubuntu系统上，Carla环境变量配置如下：
```bash
export CARLA_ROOT=/home/dut-aiid/CARLA_0.9.13
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla/agents/navigation:${CARLA_ROOT}/PythonAPI/carla:${CARLA_ROOT}/PythonAPI/examples:${CARLA_ROOT}/PythonAPI
```

## 快速开始

**1、运行 carla**

```bash
cd /home/xxx/CARLA_0.9.13
./CarlaUE4.sh
```

**2、启动 manual_control**

```
python manual_control.py -a --rolename=ego_vehicle
```

**3、运行OpenSCENARIO 2.0场景**
```
python scenario_runner.py --sync  --openscenario2 srunner/examples/cut_in_and_slow_right.osc --reloadWorld 
```