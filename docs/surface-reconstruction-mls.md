# MLS Surface Reconstruction 第一版

## 目标

第一版只生成 `Result A`：从当前 `SurfaceTargetCache` 生成与原始 iso surface 对齐的零偏移平滑面。笔刷权重、最终法线偏移和多结果比较暂不参与这一版，以便单独验证平滑质量。

## 拟合

每个 source topology 顶点使用现有 SurfaceTarget BVH/空间分组筛选局部样本。core 样本是主要位置约束，transition 样本只作为低权重的几何稳定项，不直接把零面推向透明过渡层。样本在最近 core 的切平面中展开，使用紧支撑 Wendland 核进行归一化加权二次 MLS；矩阵使用固定规模正则化求解，避免为每个查询点动态分配线性代数对象。

对 fog volume 的投影候选点会检查原始密度，默认要求不低于 `isoValue * 0.75`。该带宽限制用于阻止 MLS 在局部欠约束时把顶点推出原始体表，Viewer 中可通过 `Minimum fog density fraction` 调整。局部没有 MLS 支持、或者投影超过位移上限时，保留源拓扑顶点。

局部坐标使用世界单位，半径默认 5 mm。法线方向相反或不属于同一局部片层的样本不参加拟合。平面度只作为连续权重，不做硬阈值切换；邻域不足时退化为带正则的线性拟合。

法线来源由 `Surface Fit / Normal Seed` 与 `Surface Normal` 两个流程控制：前者使用 3×3、5×5 或 9×9 的连接邻域生成拟合种子法线，后者可选择 `None` 或继续执行 3×3、5×5 的连接趋势平均。完成 Build / update 后，Reconstruction 读取最终法线场；法线流程未构建或参数刚改变时，仍会安全回退到 SurfaceTarget 的 raw normal。

## 网格化

最终网格始终使用源 iso surface 的闭合拓扑，默认 source topology adaptivity 为 1.0，以控制三角形数量。每个源拓扑顶点沿 MLS 拟合场进行有限次受约束投影；默认执行 2 次投影，累计位移上限为 2 mm。投影步长、累计位移和 fog 密度均受保护，局部 MLS 无效或投影越过密度下限时保留原顶点。输出按现有 `SurfaceMesh` 接口三角化，因此法线模式不会再改变网格拓扑。

## 偏移顺序

MLS 结果先保持零偏移，并在每个局部拟合中消除常数项造成的锚点偏移。后续 Result B 才读取笔刷/透明度产生的偏移场，并沿 Result A 的平滑法线执行 `p_final = p_smooth + offset * normal_smooth`。

## 验证

当前首轮自动测试使用合成分层平面，检查非空网格、三角形索引和单位法线；球面、带噪圆柱和真实数据的几何误差检查作为后续质量验证项。

当前实现已经加入 `vdb_surface_reconstruction_info` 诊断入口。它会在不启动 Viewer 的情况下提取 SurfaceTarget 并生成 Result A；source topology 顶点投影按批次使用 TBB 并行执行，避免逐点串行拟合。

在 `G:\transformed\rightArm\rightArm.0224.vdb`、`density`、iso 255、MLS 半径 5 mm、最低 fog 密度比例 0.75 的实测结果为：717,147 个 core 样本，固定 source topology 输出 304,521 个顶点和 608,782 个三角形。以 2 mm 最大累计投影位移运行时，3×3 法线拒绝 162 个投影，5×5 法线拒绝 399 个投影；两种模式都保持 boundary edge 和 non-manifold edge 为 0。可用以下命令复现：

```text
build-ninja\vdb_surface_reconstruction_info.exe G:\transformed\rightArm\rightArm.0224.vdb density 255 5
```

## Viewer 显示

进入 `Surface Reconstruction` 后，`Hide original mesh` 可以切换参考 mesh；Result A 默认使用不透明、深度测试材质。`Viewer` 面板中的 `Light azimuth` 覆盖 0–360°，`Light elevation` 覆盖 −90–90°。两个角度描述“虚拟光源指向模型中心目标点”的光线方向，代码传给 Filament 时取反为表面指向光源的向量，因此可覆盖完整球面。
