# Brick Arena 模块执行文档

## 输入事实

样例文件为 `G:\transformed\rightArm\rightArm.0224.vdb`。

文件包含 `density` 与 `Cd` 两个 Grid。`density` 是 `FloatGrid` 类型的 Fog Volume，体素尺寸为 `0.00031 × 0.00031 × 0.001`，共有 56,604 个 `8³` Leaf 和 25,295,697 个活跃体素。当前模块只读取指定的浮点 Grid，不解释数值的物理含义。

## 第一版职责

第一版负责把 OpenVDB 浮点 Grid 转换成适合反复邻域计算的连续 Brick Arena，并支持重新生成等价的 FloatGrid。

模块包含以下能力：

- 按 Morton key 排列 `8³` Brick；
- 连续、64 字节对齐地保存当前值与下一轮值；
- 使用位图保存每个体素的 Active 状态；
- 预计算每个 Brick 的 26 邻接 Brick 索引；
- 保留背景值、Grid 名称、Grid Class 和 Transform；
- 验证导入后的体素值、Active 状态与原 Grid 一致；
- 可选地把 Arena 写回 VDB 文件。

## 内存结构

热数据采用 Brick 级 SoA：

```text
BrickMeta[brickCount]
currentValues[brickCount][512]
nextValues[brickCount][512]
activeWords[brickCount][8]
```

`currentValues`、`nextValues` 和 `activeWords` 分别连续分配。一个计算任务拥有完整 Brick 的写权限，后续 stencil 算法通过 `BrickMeta::neighbors` 获取邻居，不在内循环查询 VDB Tree。

## 公共接口

`BrickArena::fromGrid` 完成一次性导入和邻接构建。

`BrickArena::toGrid` 把当前缓冲区生成新的 `FloatGrid`。

`BrickArena::validateAgainst` 检查 Leaf origin、Active mask 和体素值，返回结构化验证结果。

`BrickArena::swapValueBuffers` 供后续迭代算法以常数时间交换输入输出缓冲区。

## 验证标准

合成测试覆盖负坐标、相邻 Brick、缺失邻居、Active mask、数值复制和回写。

样例文件验证必须满足：

- Brick 数为 56,604；
- 活跃体素数为 25,295,697；
- 导入验证无缺失 Brick、无多余 Brick、无 Active 状态差异、无数值差异；
- 所有热数据缓冲区满足 64 字节对齐。

## 后续扩展边界

Fog Volume 到各向同性 SDF 的转换、多尺度金字塔、Alpha 约束和曲率平滑不属于第一版。它们将在本模块提供的连续缓冲区和邻接表上实现，避免改变本次无损数据层的职责。
