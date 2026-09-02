# Viewer 架构迁移执行文档

## 目标

将 `src/viewer.cpp` 从功能实现文件收敛为 Viewer 应用组装层。算法数据、工作流阶段、Filament 展示资源和 ImGui 面板分别拥有稳定边界，同时保持现有 Surface Validation、Surface Target、Weight Painting、Reconstruction 和 Review 的行为不变。

## 模块边界

### DocumentSession

`DocumentSession` 保存当前输入文件、VDB grid、Reference mesh、SurfaceTarget cache、权重场和 Reconstruction 结果。它不依赖 Filament、SDL 或 ImGui；数据缓存和算法模块只通过它提供的数据进行读写。

### ViewerContext

`ViewerContext` 保存 Engine、Scene、View、CameraController、`LightingController` 和统一的场景坐标变换。所有渲染模块通过该上下文访问 Viewer 运行时资源，避免在多个文件中重复实现坐标换算和灯光更新。

### WorkflowController

`WorkflowController` 保存当前阶段，提供阶段切换和名称解析；阶段进入、退出和展示组合仍由 Viewer 临时编排，后续迁移到各 Stage 时再收拢局部状态。它不直接创建 Filament buffer，也不绘制 ImGui 控件。

### PresentationController

`PresentationController` 接收工作流状态，计算 Reference mesh、Result mesh、SurfaceTarget 和 Weight Painting heatmap 的最终可见性，再调用各 Renderer 的 `setVisible`。Cursor 的显示还依赖当前拾取状态，因此由笔刷阶段在每帧更新；阶段 UI 不直接修改 Scene 中的 Entity。

### Renderer modules

每个 Renderer 独立拥有自己的 Material、MaterialInstance、VertexBuffer、IndexBuffer 和 Entity，并提供 `rebuild`、`setVisible`、`destroy` 接口。当前已抽取 `MeshRenderer`、`SliceRenderer`、`BrushCursorRenderer` 和 `BrushHeatmapRenderer`；已有的 `SurfaceTargetPreview` 作为同类模块保留。`LightingController` 独立管理天空盒、间接光和方向光，不属于几何 Renderer。

## 第一阶段范围

第一阶段只处理共享基础设施：

1. 抽取 `SceneCoordinateMapper`，统一 Reference、SurfaceTarget、BVH 和结果 mesh 的场景坐标变换。
2. 抽取 `MeshRenderer`，管理 Reference/Result mesh 的 Filament 资源和材质切换。
3. 抽取 `PresentationController`，集中处理阶段可见性和 Weight Painting 的强制 Reference mesh 规则。
4. 保留现有 ViewerState 作为临时数据容器，避免在一次改动中迁移所有算法状态。

## 第二阶段范围

第二阶段抽取 `SliceRenderer`。它只接受已经完成栅格化的 `DensitySlice`，负责创建、替换和销毁 Filament 纹理；切片轴、固定索引、显示范围、轮廓开关和 ImGui tooltip 仍由 Viewer 临时维护。这样 Slice Comparison 的数据逻辑和展示资源已经分离，后续可以再将切片面板迁移到独立阶段控制器。

## 第三阶段范围

第三阶段抽取 Weight Painting 的展示资源和请求组装。`BrushCursorRenderer` 负责屏幕朝向的 core/falloff 双环几何、物理尺寸和相机朝向更新；`BrushHeatmapRenderer` 负责表面顶点到 VDB voxel 的映射、权重颜色、局部三角索引和 Filament entity 生命周期；`WeightPaintingStage` 负责将 UI 的毫米/角度参数转换为表面笔刷请求。Viewer 只负责拾取、调用表面笔刷算法、维护权重场和决定展示时机。

## 后续迁移顺序

完成渲染资源拆分后，再依次迁移 `WeightPaintingStage`、`ReconstructionStage`、Validation UI 和 Workflow UI。每一步都必须先保持 headless smoke 和现有阶段切换行为，再删除 viewer.cpp 中对应的旧实现。

## 验证标准

构建验证包括 MSVC/Ninja 编译、当前 rightArm VDB 的 headless smoke，以及以下显隐规则：Surface Target 使用独立 Reference mesh 状态，Reconstruction/Review 使用原始 mesh 状态，Weight Painting 始终显示 Reference mesh 但不改写其他阶段状态。

## 第四阶段：数据持久化边界

本阶段只迁移不依赖 Filament 和 ImGui 的持久化逻辑：

1. `WeightFieldRepository` 管理权重场目录、文件命名、VDB metadata、兼容性检查、保存和加载。
2. `BrushSettingsStore` 管理全局 brush 参数的 JSON 读写。
3. `BrushProfileRecorder` 管理 JSONL 轨迹、拟合记录和回放文档。

这些模块通过显式的输入结构和结果结构与 Viewer 交互，不接收 `ViewerState&`。Viewer 仍保留状态同步、错误提示和触发时机，确保本阶段不改变 UI 行为。

## 第四阶段验证

除常规构建和核心测试外，必须继续运行 rightArm headless smoke，确认权重场保存/加载 round-trip、brush profile 回放和首次加载默认设置行为不变。

## 第四阶段实现状态

`WeightFieldRepository`、`BrushSettingsStore`、`BrushProfileRecorder`、`SurfaceTargetCacheRepository`、`ViewerOptions`、`DocumentSession` 和 `ViewerState` 已接入 viewer。viewer 只保留状态同步、阶段触发、错误提示和渲染调用；VDB/JSON/JSONL 的格式细节以及缓存身份校验位于对应模块中。

当前仍留在 viewer 的下一批边界是输入文档状态、刷笔流程调度和 ImGui 窗口。这些部分依赖多个阶段状态，下一轮再按“先数据容器、后 UI”迁移，避免把渲染或交互回调反向塞入持久化模块。

## 第五阶段：应用配置与状态声明

本阶段先移动两个低风险边界：`ViewerOptions` 负责命令行参数及默认输入，`ViewerState` 负责 Viewer 的运行时状态声明。两者不改变状态字段语义，也不移动算法和 ImGui 逻辑；viewer 继续负责阶段编排和状态更新。这样后续才能把 `DocumentSession`、`BrushInteractionState` 和各阶段 UI 从同一个巨型结构中逐步分离。

## 第五阶段验证

除核心测试和 viewer 构建外，必须用默认参数与显式 `--input/--grid/--iso/--adaptivity` 各运行一次 headless smoke，确认参数解析、缓存命中和现有渲染初始化行为一致。

## 第六阶段：DocumentSession

`DocumentSession` 只保存输入 VDB、表面层级加速结构、Surface Target cache、权重场和 Reconstruction 数据。它不持有 Filament entity、SDL 输入状态或 ImGui 控件状态。首版通过 `ViewerState` 的继承保持已有 `state.grid` 等访问兼容，后续再按阶段拆分引用，避免一次性重写所有 UI 回调。

## 第六阶段验证

要求 viewer 构建、核心测试、默认/显式参数 headless smoke 的输出与第五阶段一致，重点检查 Surface Target cache 命中、权重场 round-trip 和 Reconstruction 的状态字段仍可用。

## 第七阶段：BrushInteractionState

本阶段只迁移笔刷交互的瞬时状态，不改变笔刷算法、渲染资源、持久化数据或 ImGui 绘制逻辑。状态包括 GPU pick 请求、Ctrl/鼠标按键状态、指针位置、笔刷路径采样与拟合进度、拾取视口变换、cursor 命中位置，以及数值框精度拖动状态。

首版通过 `ViewerState` 继承 `BrushInteractionState` 保持现有 `state.brush...` 访问兼容，同时将字段声明移出 `ViewerState.h`。这样不会在一次改动中引入大量成员访问改写；后续抽取 `BrushInteractionController` 时，再把这些字段改为显式的状态对象引用。

## 第七阶段验证

除 viewer 构建、核心测试和默认/显式参数 headless smoke 外，必须检查 Ctrl+左键笔刷 pick、连续路径采样、松开鼠标后的 finalize，以及数值框精度拖动状态仍可用。headless smoke 的输出指标应与第六阶段一致。

## 第七阶段实现状态

`BrushInteractionState` 已从 `ViewerState` 中抽出，当前通过继承保留旧访问路径。笔刷设置、SurfaceBrushResult、heatmap/cursor renderer、profile recorder 和 UI 状态仍留在 `ViewerState`，作为下一步 controller 与面板拆分的边界。

## 第八阶段：BrushInteractionController

本阶段把笔刷交互的行为从 viewer 组装层移出：GPU pick 坐标换算与回调、路径中心去重、延迟拟合、单点笔刷处理、连续路径处理、profile 记录和 profile replay 统一由 `BrushInteractionController` 负责。Controller 只接收 `ViewerState`、Filament Engine/Scene/View，并通过现有状态和 Renderer 完成更新；SurfaceBrush 算法本身不在本阶段改写。

热力图应用和权重场合并也作为 controller 的共享操作暴露给 viewer 的权重场加载逻辑，避免迁移后复制两套结果同步代码。参数保存、权重场文件管理和 ImGui 控件继续留在原模块，确保这次迁移只改变行为归属，不改变数据格式或界面语义。

## 第八阶段验证

要求 viewer 构建、默认/显式参数 headless smoke、核心测试全部通过；smoke 必须继续覆盖两次单点 brush、权重场 round-trip、cursor renderable，以及 profile replay 的已有输出。交互事件在运行时仍由 `ViewerDisplayManager` 收集，但只负责更新输入状态并调用 controller 的 profile 起止操作。

## 第八阶段实现状态

`BrushInteractionController` 已接入 `ViewerState`。viewer 中原有的笔刷 pick、路径拟合、profile replay 和结果同步函数已迁移到独立实现文件；viewer 仅保留阶段调度、UI 触发和状态展示。

## 第九阶段：BrushPaintingPanel

本阶段把 Weight Painting 窗口、数值框精度拖动、笔刷参数编辑、权重场库操作和 brush preset 操作迁移到 `BrushPaintingPanel`。Panel 只通过 `ViewerState` 和现有 controller/Renderer 操作数据，不直接管理工作流切换或 Filament entity 生命周期。

Viewer 保留一个无状态的 Panel 实例，并在 Weight Painting 阶段调用 `draw`；headless smoke 所需的权重场创建、保存、加载和热力图恢复接口也由 Panel 提供，避免 Viewer 继续依赖笔刷面板内部的持久化细节。

## 第九阶段验证

要求 viewer 构建、默认/显式参数 headless smoke、核心测试全部通过；smoke 必须继续覆盖权重场保存/加载 round-trip。交互验证重点是数值框中键精度选择、离开精度条后的水平拖动、Clear/New/Load/Save/Discard 以及 preset 保存加载。

## 第九阶段实现状态

`BrushPaintingPanel` 已接入 `ViewerState`，Weight Painting 窗口及其权重场/preset 操作已从 `viewer.cpp` 移出。Viewer 仅负责阶段选择和调用 Panel。

## 第十阶段：WorkflowPanel

本阶段迁移 Workflow 窗口的完整 ImGui 绘制边界，包括阶段节点、顺序/数据依赖箭头、节点提示，以及窗口内的 Viewer 缩放和灯光参数。`WorkflowPanel` 不直接执行阶段切换，也不调用 `PresentationController`；点击节点时只返回请求的 `WorkflowStage`，由 Viewer 在统一编排位置调用既有的 `setWorkflowStage`。

方向光角度到 Filament 光照方向的转换也归属于 `WorkflowPanel`，从而窗口中的灯光控件和其数学映射保持在同一个模块。Viewer 初始化只调用该模块公开的方向计算接口，不再保留 Workflow 窗口的绘图细节。

## 第十阶段验证

要求 viewer 构建、默认/显式参数 headless smoke、核心测试全部通过；交互验证重点是流程节点的 ready/locked/active 状态、节点点击后的阶段切换、顺序箭头和数据依赖箭头，以及 Viewer 缩放、全局光照和方向光角度控制不变。`viewer.cpp` 只保留 Panel 调用和阶段副作用编排。

## 第十一阶段：ReconstructionPanel

本阶段迁移 Surface Reconstruction 窗口的参数编辑、Result A/B/C 状态和重建统计展示。面板不直接接收 Engine/Scene，也不创建或销毁 GPU 资源；“Generate Result A”和“Clear Result A”只返回动作请求，由 Viewer 调用既有的重建函数和资源生命周期操作。这样重建算法、渲染资源和 UI 的边界保持清晰，同时保留原有参数范围与状态文本。

## 第十一阶段验证

要求 viewer 构建、默认/显式参数 headless smoke、核心测试全部通过；交互验证重点是 MLS 参数修改、生成 Result A、清除 Result A、原始 mesh 显隐，以及重建计时和候选单元统计显示不变。
