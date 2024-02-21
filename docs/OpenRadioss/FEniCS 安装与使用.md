## FEniCS 安装与使用
### 安装二进制

        add-apt-repository ppa:fenics-packages/fenics
        apt update
        apt install fenicsx

### 安装可视化

        apt install libgl1-mesa-glx xvfb

### 安装conda

* 下载 Anaconda
  
        wget https://mirrors.bfsu.edu.cn/anaconda/archive/Anaconda3-2022.10-Linux-x86_64.sh --no-check-certificate

* 安装 

        bash Anaconda3xxxxxxx.sh

* 环境变量

        source ~/.bashrc

* 升级（可选）

        conda update -n base -c defaults conda

* 创建虚拟环境

        conda creat -n fenicsx-env
        conda activate fenicsx-env
        conda install -c conda-forge fenics-dolfinx mpich pyvista

* 使用该环境运行代码，比如 官网例子 或 test.ipynb
