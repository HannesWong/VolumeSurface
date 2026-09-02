# 笔刷性能记录与回放

## 目的

`brush_profile.jsonl` 是自动追加的笔刷性能记录。每一行对应一笔已经完成的连续笔画，既保存原始表面锚点和相对时间，也保存每次预览拟合的输入快照与耗时。它用于比较不同算法版本在完全相同笔画负载下的开销。

## 记录时机

按住 `Ctrl` 左键绘制时，Viewer 记录被接受的 GPU 拾取世界坐标。每次 120 ms 节流拟合和松开鼠标后的最终拟合都会成为一个 fit 记录。笔画完成时，Viewer 将整个 stroke 作为一行 JSON 追加到当前工作目录的 `brush_profile.jsonl`。

## 内容

每个 stroke 记录包含：

- VDB 文件和 grid 名称；
- density iso 值和完整笔刷参数快照；
- 每个锚点相对笔画开始的毫秒时间与世界坐标；
- 每次 fit 使用的锚点数量与是否为最终 fit；
- BVH 查询、block、anchor resolve、centerline route、surface sweep、heatmap 和总耗时；
- 候选体素数、候选 leaf 数、中心线节点数、sample 数和热力图三角形数。

坐标始终使用 VDB 世界单位；当前界面的毫米参数同样原样保存，回放时再按同一换算生成算法参数。

## 回放

使用下面的命令在隐藏的 Viewer 中重放文档：

```powershell
build-viewer\volume_surface_viewer.exe --input G:\transformed\rightArm\rightArm.0224.vdb --replay-brush-profile brush_profile.jsonl
```

回放按记录中的每次 fit 所对应的锚点检查点依次执行，不等待真实时间间隔，因此输出只反映计算和提交成本，不会被用户停顿时间污染。原始相对时间仍保留在文档中，用于之后检查输入节奏。

增量版本会从上一次检查点的最后一个锚点开始重放，而不是重算整条前缀。控制台中的 `anchors` 是笔划累计锚点数，`segment_anchors` 才是这一轮新尾段实际使用的锚点数。

## 比较规则

同一份文档必须配合同一 VDB、grid 名称、iso 值和 mesh adaptivity 使用。一次优化后重新运行相同命令，将控制台输出中的 `replay.fit` 与旧记录的 timing 字段比较。

从整笔前缀改为尾段增量传播后，单轮的候选体素数、中心线节点数和 sample 数会下降；应比较整笔累计覆盖范围与最终热力图三角形数，而不是要求每轮的这些数值相同。`schema: 1` 的既有记录仍可读取；新记录使用 `schema: 2` 并写入 BVH 查询时间和候选 leaf 数。
