## 虚幻引擎管线

### 为什么需要修改管线

1. 只有修改了管线才可以让 SDF，SShairShadow，VirtualShadowMap，SelfShadowMap完美融合，和引擎的原装逻辑完美契合。
2. 主要还是性能问题，比如说常见的后处理卡通渲染为了后处理阶段拿到更多参数，向 Gbuffer 里传值，Shading Mode 就得用 Default Lit，在后处理阶段又得把 PBR 光照计算结果用卡渲效果再覆盖掉，一堆无意义的性能开销，只能说停留在demo阶段，或者当材质练习。