# Surface Target 预览

## 观察对象

Surface Target 阶段需要同时观察三个层次：表面点、点的法线和空间分组。当前点源是 `SurfaceTargetCache`：alpha=255（或当前 isoValue）边界作为 core，向透明侧最多扩展两层作为 transition。Reference mesh 仍保留用于轮廓对照，不再作为正式表面样本的唯一来源。

## Viewer 图层

左侧面板提供独立开关：`Surface mesh` 控制当前 Surface Target 阶段的 Reference mesh，`Point cloud`、`Surface normals` 和 `BVH bounds` 控制预览图层。点和法线使用稀疏采样，避免把八十多万个 Reference 顶点全部作为调试图元提交到渲染器。采样步长可以在面板中调整，改变后只重建对应调试缓冲。

BVH 使用实际笔刷层级中的节点边界，而不是重新构造一个显示用树。面板中的 depth 选择某一层节点；较浅层用于观察整体分组，较深层用于检查局部叶节点。每个盒子显示其 terminal 状态和包含的 VDB leaf 数量。

## 坐标和性能

所有调试图层沿用 Reference 的 `referenceCenter`、`displayScale` 和 Viewer 场景偏移，因此点、法线和 BVH 盒子与体表重合。调试缓冲只在开关、采样步长或 BVH depth 改变时重建；阶段切换只改变实体显隐，不重复上传未变化的数据。

## 平面原始表面

Slice Comparison 保持为平滑前的诊断视图：灰度背景来自当前 density slice，`Raw iso surface` 是当前 isoValue（默认 255）的原始等值线，`Core samples` 和 `Transition samples` 是同一切片上的 SurfaceTarget 体素采样投影。core 点优先覆盖同一像素，transition 点只作为辅助提示；这里不显示 MLS 或任何重建结果。

## 后续替换点

后续 MLS/隐式场直接读取 `SurfaceTargetCache`，Viewer 图层和 BVH 检查方式保持不变。过渡样本使用离最近 core 的层数计算连续支持权重，法线预览也直接显示缓存中的世界空间法线。

当前平面度采用每个 core 的 26 邻域快速估计，`planarityRadius` 作为物理距离阈值和权重尺度；更大半径的测地邻域留给后续重建阶段，以免首次加载把每个点扩展成大体素立方体扫描。

## 独立点云预览模块

Surface Target 的 GPU 预览由 `SurfaceTargetPreview` 模块负责。Viewer 只传入 SurfaceTarget cache、VDB voxel size、场景坐标变换和 BVH 指针，并调用模块的 rebuild、visibility 和 destroy 接口；Filament buffer、material、entity 以及采样缓冲不再由 Viewer 直接管理。

点云预览使用 cache 中的世界空间法线进行方向光照。点的显示半径是渲染参数，不会修改 `worldPosition`、`normal`、`density`、`supportWeight` 或 Reconstruction 输入。默认渲染尺度以 VDB 的物理 voxel size 为基准，并允许使用无量纲 `pointScale` 调整可见性。

当前 cache 只有 `Core` 和 `Transition` 两种样本类型。预览过滤使用 `Core`、`Transition`、`Both` 三种显示状态，不增加第三类样本，也不通过框选修改 cache 分类。法线在 cache 构建阶段计算并持久化；后续预览只读取缓存。

点云使用深度测试和物理 point-sprite 尺寸。由于 POINTS 没有三角形正反面，朝向过滤在材质中按 `dot(normal, pointToCamera) > 0` 执行；这表示只显示朝向相机的点。过渡点与 core 点可独立显示，以避免重叠遮挡误读。

法线线使用透明混合、深度测试和不写深度；线端点携带缓存法线，在材质中执行同样的朝向过滤。BVH 线复用该材质但关闭法线过滤，因此仍可完整观察空间分组。

法线线的表面根部使用蓝色，沿外法线方向的末端使用亮红色；颜色沿线段插值，可直接辨认法线方向和两端。

Reference mesh 的可见性按工作阶段分别保存：Surface Target 使用自己的 `Surface mesh` 状态，Reconstruction/Review 使用 `Show original mesh` 状态；Weight Painting 为了保证笔刷定位始终强制显示 Reference mesh，但不会改写另外两个阶段的勾选值。
