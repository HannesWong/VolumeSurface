# Surface Target 缓存

## 文件位置

Surface Target 使用与输入 VDB 同目录的 sidecar 文件保存，不压缩样本数据。几何样本文件名为 `<输入文件名去扩展名>.<grid 名>.surface-target.bin`，例如 `rightArm.0224.density.surface-target.bin`。局部翻转点作为同一 cache package 的独立 JSONL 文件保存：`<输入文件名去扩展名>.<grid 名>.surface-target.local-seeds.jsonl`（文件名保留以兼容已有缓存）。

## 自动流程

启动时先读取 sidecar。只有来源路径、文件大小、写入时间、grid 名称、grid class、活动包围盒和 Surface Target 参数全部一致时才接受缓存；任一项变化都会重新提取表面点，并在提取成功后自动覆盖保存。面板中的 `Rebuild surface target` 会强制重新生成并保存，`Save cache` 与 `Load cache` 用于手动控制。

## 数据内容

二进制缓存保存 core/transition 样本的坐标、世界坐标、法线、密度、平面度、角度偏差、支持权重、过渡层数和样本类型，同时保存用于失效判断的元数据。格式带有 magic 和版本号，加载时会校验样本数量、分类计数和元数据；损坏或过期的文件不会替换当前内存中的缓存。局部翻转点文件只保存坐标和目标方向，并复用来源、grid、isoValue、transitionLayers、源文件大小与写入时间做失效判断。

## 兼容性边界

当前格式是本地小端二进制格式，目标是同一工程和同一数据源之间的快速复用，不承诺跨平台或跨版本长期归档。首次生成仍需执行完整的表面点提取；后续启动可以跳过这一步，加载耗时取决于磁盘吞吐和样本数量。

## 法线种子

缓存版本 4 保存基于体素单元三线性梯度的法线种子。Core 样本先沿六个轴向寻找等值面交点，再分别在每个有效交点处计算梯度并合并方向；Transition 样本在自身的三线性采样位置计算梯度。该过程不使用物理半径选择邻域，也不改变 3x3/5x5 的 Surface Normal 邻域规则。

如果某个交点的三线性梯度退化为零，只在该样本上使用中心差分作为数值安全兜底，不改变正常样本的法线估计策略。缓存同时保存 planarity、角度偏差和 support weight 等辅助元数据。

`normalRadius` 字段暂时保留在元数据中以兼容已有接口，但不再参与法线计算。版本号变化会使旧 sidecar 自动失效，避免旧的平滑法线混入新的连通趋势流程。

## 局部 flip point 自动缓存

局部 flip point 在 Local Flip Points 面板中执行 Add 后立即写入 JSONL、启用并重放；`Replay selected flip point` 可再次执行同一点的回放；删除 flip point 后也立即重写该小文件。应用加载几何缓存后会自动读取有效的局部 flip point，按保存顺序在全局 seed 基线之上重放，并重新计算 affected/boundary 区域。affected/boundary 点集不落盘，避免平滑参数或拒止边界策略变化后继续使用过期结果。旧文件中的 `seed` 记录仍可读取，但会作为 legacy inactive 保留，不会自动参与翻转；新保存的记录使用 `flip_point`。

## 局部 flip point 诊断参数

`surface_normal_diagnostics_info` 的第 5–8 个可选参数依次覆盖局部最大 Core 数量、局部最大距离（mm）、局部最小轴向对齐度和表面连续性分量阈值。诊断输出会同时给出每类拒止原因和是否撞到最大 Core 数量上限，用于区分物理距离限制、几何连续性限制与紧急截断。
