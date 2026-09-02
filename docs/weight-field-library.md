# 权重场库

## 目标

权重场库保存已刷出的局部表面影响，而不是依赖当前网格顶点顺序的显示缓存。加载同一来源的权重场后，用户可以修改全局生成参数并重复生成不同结果，无需重新绘制笔刷。

## 文件组织

每个输入 VDB 在其同级目录中拥有一个独立库目录：`<source-stem>.weight-fields`。目录内的每个 `*.brushmask.vdb` 文件都是一个可切换的权重场；原始 VDB 不会被修改。

默认新建权重场名为 `default.brushmask.vdb`。面板允许编辑名称后另存为，并可刷新目录以发现由外部程序或其他会话创建的文件。

`New empty` 创建未命名的空场；它不修改已保存文件，直到用户显式 Save 或 Save As。

## VDB 内容

每个权重场文件包含名为 `surface_weight` 的稀疏 `FloatGrid`。它使用 Reference density grid 的 transform，背景值为零，并且只激活权重大于零的表面体素。

文件元数据记录 schema 版本、来源 grid 名称和来源 active bbox。加载时，transform、grid 名称和 active bbox 都必须与当前 Reference 匹配；不匹配的文件不会覆盖当前场。

## 面板状态

面板同时维护“当前已加载场”和“库中选中的候选文件”。改变候选文件不会自动加载；只有点击 Load 才替换当前权重场。

刷写或清空会使当前场变为 dirty。dirty 状态下禁止 Load，用户必须先 Save、Save As 或 Discard，避免静默丢弃已刷修改。Refresh 不改变当前场。

## 运行时映射

笔刷传播结果首先合并到 `surface_weight`，再按已有的表面体素到 mesh 顶点映射更新热力图。加载或重建 Reference 时，热力图从权重场重建一次；常规笔刷不执行全量重建，以避免增加拖动延迟。

## 后续扩展

未来局部平滑参数应作为同文件中的额外稀疏 grid 保存，例如 `surface_smoothing_radius_mm`。本阶段只读写 `surface_weight`，不提前写入尚未定义语义的字段。
