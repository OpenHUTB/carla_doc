# 行人物理场模拟

## 强化学习

```shell
conda create -n opensim-rl -c kidzik opensim python=3.6.1
conda activate opensim-rl
conda install -c conda-forge lapack git
pip install git+https://github.com/stanfordnmbl/osim-rl.git
python
```

测试代码：
```python
from osim.env import ProstheticsEnv
env = ProstheticsEnv(visualize=True)
observation = env.reset()
for i in range(200):
    o, r, d, i = env.step(env.action_space.sample())
```

## 参考

- [Chrono OpenSim 解析器手册](https://sbel.wiscweb.wisc.edu/wp-content/uploads/sites/569/2018/06/TR-2017-08.pdf)

- [OpenSim解析器的C++类文档](https://api.projectchrono.org/classchrono_1_1parsers_1_1_ch_parser_open_sim.html)

- [带肌肉骨骼的强化学习环境](http://osim-rl.kidzinski.com/)
