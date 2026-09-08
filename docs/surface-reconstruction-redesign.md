# 表面重建重新设计

## 当前基线

`SurfaceMesh::extractIsoSurface` 当前使用 OpenVDB `VolumeToMesh` 生成 Reference mesh。它适合 Viewer 显示和结果对比，但不应被当作最终平滑算法的点源，因为其顶点密度和三角形布局已经受到原始等值面网格化的影响。

`SurfaceBrush` 已经包含基于 VDB 的表面分类、世界空间法线、局部平面度、26 邻域传播和一次构建的 VDB leaf BVH。权重场以稀疏 `surface_weight` FloatGrid 保存。流程控制和 Surface Target Viewer 也已经存在。

## 新的核心数据流

```text
Source VDB
    │
    ├── SurfaceTargetCache ────────────────┐
    │                                      │
    └── Slice Comparison（独立检查工具）   │
                                           ▼
Weight Field ───────────────────────► Surface Reconstruction ─► Result Mesh
```

`SurfaceTargetCache` 是重建和笔刷共同使用的表面样本缓存，不再让重建算法从 Reference mesh 反推点集。当前每个样本包含世界坐标、世界法线、VDB 坐标、density、平面度、角度偏差、过渡层级和空间权重；空间裁剪信息由共享的 leaf BVH 提供。

## 模块职责

### SurfaceMesh

保留 `VolumeToMesh` 作为 Reference 和临时结果的显示转换器。它不负责表面样本提取，也不负责最终平滑。

### SurfaceTarget

新增独立模块，负责从 VDB 提取 alpha=255（或当前 isoValue）边界和透明过渡样本、计算法线与平面度，并提供采样、统计和 Viewer 调试数据。当前 Viewer 已经使用该缓存作为 Surface Target 点源；Reference 顶点仅用于轮廓对照。空间裁剪仍复用 `SurfaceBrushHierarchy` 的 leaf BVH。

### SurfaceBrush

继续负责路径、笔刷传播和局部连续性判定。现有的表面分类、法线和局部平面度计算应迁移到 `SurfaceTarget` 或共享缓存，避免每次笔刷和每次重建重复计算。

### SurfaceHierarchy

第一阶段继续复用现有 `SurfaceBrushHierarchy` 的 leaf BVH。节点摘要增加表面样本数、法线锥、平面度上下界和空间权重范围，使重建可以在节点级拒绝无关区域。只有当单个 leaf 内样本数成为瓶颈时，才在 leaf 内建立点级 BVH，不预先复制一棵全局点 BVH。

### WeightField

将当前位于 Viewer 中的保存/加载逻辑提取为可复用模块。现有 `surface_weight` 保持兼容；支持局部 core radius、falloff radius 和 strength 后，schema v2 增加对应稀疏字段或一个打包的参数 Grid。重建阶段直接读取这些字段，而不是只使用热力图顶点映射。

### SurfaceReconstruction

重建模块接收 `SurfaceTargetCache`、`WeightField` 和全局参数，执行局部拟合并输出 `SurfaceMesh`。Viewer 只负责调度、显示 Result A/B/C 和报告耗时。

## 重建算法顺序

### 无权重基线

先使用所有有效表面样本生成未刷的基线结果，验证人体轮廓、端部闭合和法线方向。这个结果只用于对比，不依赖 SDF。

### 局部拟合

在每个查询点附近建立世界空间切平面，以样本法线为局部坐标轴，使用紧支撑权重的二次 MLS 拟合。邻域半径使用物理单位，并分别处理 0.31 mm 的 XY 间距和约 1 mm 的 Z 间距。

### 参数场融合

全局参数和笔刷参数不采用硬切换。对位置 `x`，用连续权重 `w(x)` 融合半径和强度：

```text
parameter(x) = (1 - w(x)) * globalParameter + w(x) * brushParameter
```

`w(x)` 使用当前笔刷的 smootherstep 过渡；多个笔刷通过连续的 soft-max 或归一化累积合并，从而避免刷痕边界产生导数跳变。

### 连续场和拓扑

MLS 结果先形成连续的点驱动隐式场，再用 Dual Contouring 生成网格。VDB 的 inside/outside 分类只作为符号和连通性约束，不把 density 数值强行解释成 SDF。

为避免相邻但不相连的表面被桥接，节点级连通性、法线方向一致性和局部占据状态必须在隐式场采样时共同参与；不能只依赖距离阈值。

## 流程阶段映射

主流程保持：

```text
Source VDB → Surface Target → Surface Fit / Normal Seed → Local Flip Points → Weight Painting → Surface Reconstruction → Review / Export
```

Slice Comparison 继续作为独立 Inspector。Surface Target 负责确认正式样本和 BVH，Surface Fit / Normal Seed 生成法线种子，Local Flip Points 只处理局部翻转链，Weight Painting 只修改 WeightField，Reconstruction 只读取这些结果并产生网格。

## 实现顺序

1. 从 `SurfaceBrush` 提取可复用的 `SurfaceTargetCache`，先复用现有 6 邻域边界、梯度法线和平面度逻辑。
2. 把 Surface Target Viewer 的点源从 Reference mesh 顶点切换为缓存样本，并保留点、法线和 BVH 调试开关。
3. 将权重场读写从 Viewer 中抽出，设计兼容旧文件的 schema v2。
4. 实现无权重 MLS 基线和局部隐式场，输出 Result A。
5. 接入权重参数融合，输出 Result B，并保留未刷区域作为对照。
6. 最后再根据 profiling 决定是否加入 leaf 内点级 BVH和并行重建。

## 验证指标

验证重点直接对应最终目标：Reference 与基线结果的轮廓距离、刷大半径区域的曲率变化、手指等细部的保留率、刷区边界的一阶/二阶连续性、独立表面之间的误连接率、法线跳变和三角形质量。SDF 只在将来确实需要距离运算时作为可选派生数据，不作为本链路的前置验证。
