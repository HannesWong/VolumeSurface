# 流程控制设计

## 目标

流程控制层负责在已有的 VDB、表面、权重场和重建结果之间切换，不直接实现任何表面算法。切换阶段只改变参数面板与 Viewer 的展示路由，已经计算出的数据继续保留。

## 用户可见阶段

主流程采用单向的 Houdini 风格节点图：

```text
Source VDB → Surface Target → Surface Fit / Normal Seed → Local Flip Points → Weight Painting → Surface Reconstruction → Review / Export
```

`Source VDB` 显示当前输入和来源状态。`Surface Target` 用于观察将参与后续拟合的表面目标，包括 core/transition 点、世界空间法线和 BVH 状态。`Surface Fit / Normal Seed` 使用 3×3、5×5 或 9×9 的离散连通邻域拟合局部表面趋势并生成种子法线。`Local Flip Points` 负责拾取、预览和缓存局部翻转链。`Weight Painting` 使用现有笔刷和权重场。`Surface Reconstruction` 预留平滑与三角网重建参数。`Review / Export` 用于对比结果并导出。

## Slice Comparison 的位置

Slice Comparison 是全局只读检查工具，不属于主流程节点，也不产生顺序边。它可以在任意阶段打开，读取 Source VDB 以及可选的 Reference、Result A/B/C。切片参数由工具面板控制，切片平面和等值线由 Viewer 显示。

## 边的语义

流程图区分两类边：

- 顺序边使用实线和箭头，表示推荐的阶段推进顺序。
- 数据边使用虚线和箭头，表示某个阶段读取的具体产物，例如 Weight Field 到 Surface Reconstruction。

线型和箭头形状同时表达语义，不依赖颜色。流程图底部提供图例。

## 阶段状态

每个阶段具有 `available`、`active` 和 `stale` 状态。上游数据不存在时，节点显示为锁定；数据变化使下游阶段变为过期，但不会删除旧结果。点击锁定节点只显示原因，不会跳转或触发耗时计算。

## Viewer 显示路由

阶段面板不直接操作 Filament 实体，而是更新统一的 `ViewerPresentation`：

- Source VDB：Reference mesh 和输入数据状态。
- Surface Target：Reference、表面目标预览和 BVH/点云统计。
- Surface Fit / Normal Seed：Reference、拟合种子法线预览。
- Local Flip Points：Reference、局部翻转链预览。
- Weight Painting：Reference、权重热力图和笔刷光标。
- Reconstruction：Reference 与 Result A/B/C。
- Review / Export：按结果槽选择需要比较或导出的 mesh。

隐藏一个实体只修改可见性，不重复创建 GPU 缓冲，也不重建 BVH。Slice Comparison 的切片实体单独管理，不受当前流程阶段显隐策略限制。

## 第一阶段实现范围

第一阶段只实现流程状态、右侧节点图、顺序边和数据边、左侧阶段面板路由，以及独立的 Slice Comparison 保持可用。Surface Target 现在提供 `SurfaceTargetCache` 的 core/transition 点、法线和现有 BVH 检查信息；MLS 和最终重建算法继续在后续阶段实现。
