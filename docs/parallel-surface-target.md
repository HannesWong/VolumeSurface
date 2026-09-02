# SurfaceTargetCache 并行化实施方案

## 目标

启动阶段的 SurfaceTargetCache 构建必须保持输出确定，同时降低 186 万级样本提取和 71 万级 core 样本分析的 CPU 时间。并行化只作用于 CPU 数据准备；Filament 资源创建、GPU 上传和 ImGui 状态更新仍在主线程执行。

## 阶段划分

1. 扫描 active voxel，按 OpenVDB leaf 分区，每个分区写入线程局部 core 坐标列表。
2. 对每个 core 扩展 transition 坐标，每个分区使用线程局部候选列表和最近层级表。
3. 合并、排序、去重坐标，并生成稳定的样本索引。
4. 法线和样本基础属性按样本区间并行计算；每个 worker 使用独立的只读 OpenVDB accessor。
5. 建立 core 索引后，按样本区间并行计算 26 邻域平面度。
6. 对无效 transition 法线做局部 core 回填，并保持确定性顺序。

## 线程安全边界

- 不在线程之间共享可写的 `std::vector` 或 `std::unordered_map`。
- 不共享 OpenVDB `ConstAccessor`；每个 worker 创建自己的 accessor。
- 只读的 grid transform、core 索引和已完成的样本数组可以被多个 worker 读取。
- 并行阶段结束后才能进入下一阶段，避免样本索引和邻域索引未完成时被读取。

## 确定性和回退

线程局部结果在阶段边界统一拼接，再使用 `parallel_sort` 和 `unique`；因此线程调度不会改变最终坐标顺序。若 TBB 不可用，保留现有单线程路径，接口和结果格式不变。

## 预期收益

法线、平面度和 transition 扩展是主要并行候选。排序、OpenVDB 随机访问和内存分配仍可能受内存带宽限制，因此不预设线性加速。真实 VDB 以 `surface_target.samples`、core/transition 数量和 `build_ms` 作为回归指标。

## 当前实现状态

当前版本已使用 TBB 并行执行 leaf 扫描、transition 扩展、样本法线、26 邻域平面度、transition 法线回填和排序。真实 `rightArm.0224.vdb` 的样本数量保持为 1,860,755，缓存构建时间由约 27.3 秒降至约 10.4 秒。
