# 输入选择与缓存切换

## 启动入口

Viewer 传入 `--input <file.vdb>` 时直接使用该文件。省略 `--input` 时，Viewer 读取工作目录、可执行文件目录或其上级目录中的 `viewer_inputs.jsonl`，显示输入路径下拉框；当前配置将 `G:\images\male.vdb` 作为首项和默认项。确认选择后才加载 VDB，取消选择会终止本次启动，不会创建半初始化的 Viewer。

每行使用一个独立 JSON 对象，例如：

```jsonl
{"label":"male","path":"G:\\images\\male.vdb","default":true}
{"label":"rightArm.0224","path":"G:\\transformed\\rightArm\\rightArm.0224.vdb"}
```

相对路径以 JSONL 文件所在目录为基准；不存在的路径不会进入下拉框。配置文件缺失或没有有效项时，程序会退回内置的 male/rightArm 候选。

`--inspect-only`、`--headless-smoke` 和 `--replay-brush-profile` 不弹出选择框，仍使用 rightArm 样例作为自动化回归输入。这样不会让原有脚本因为新增交互入口而卡在系统文件对话框中。

## 错误日志

Viewer 会将启动阶段、输入文件、grid 读取、初始 mesh、Surface Target 和 Viewer 创建等事件追加到同目录的 `viewer_error.jsonl`。正常关闭也会写入结束记录；标准异常、`std::terminate` 和 Windows 未处理异常会分别记录错误阶段、异常消息或异常代码。双击启动时没有可见控制台，可以直接查看这个 JSONL 文件定位最后完成的阶段。

## 当前缓存

缓存属于“输入文件 + grid + 参数”的 sidecar，而不是全局共享对象。当前已经存在的 rightArm 缓存包括：

- `rightArm.0224.density.surface-target.bin`：Surface Target 点、法线、分类、支持权重和 BVH 依赖的数据。
- `rightArm.0224.density.surface-target.local-seeds.jsonl`：局部 flip point 列表；affected/boundary 不落盘，会在当前缓存上重放得到。
- `rightArm.0224.density.surface-normal-seed.jsonl`：全局法线种子。
- `rightArm.0224.weight-fields\*.brushmask.vdb`：以输入 stem 分目录保存的权重场。

`G:\images\male.vdb` 当前已经生成了 `male.density.surface-target.bin` 和 `male.density.surface-normal-seed.jsonl`。局部 flip point 和权重场仍只有在对 male 执行相应操作并保存后才会出现。Viewer 运行期间的网格、法线传播、重建结果和 Filament 资源都只属于当前输入，不会跨文件复用。

## 加载与失效规则

Surface Target 和局部 flip point 都校验规范化源路径、文件大小、写入时间、grid、活动包围盒以及相关参数。全局种子校验源路径、grid 和 iso；权重场校验源 grid、活动包围盒和 transform。任一校验不通过时只放弃该文件的缓存并重新构建，不会把另一 URL 的结果套到当前数据上。

## 不同 URL 的切换

当前版本已经支持“选择后进入”：输入在 Viewer 初始化前确定，所以不会留下旧数据的 GPU 资源或算法状态。尚未在已打开的窗口内加入热切换按钮；要做热切换，需要将当前流程实现为一次完整的 document transaction：停止异步任务、销毁旧 renderer、清空 Surface Target/种子/权重/重建状态、加载新 VDB，再按新 URL 的 sidecar 逐项恢复。这个边界明确后，male 与 rightArm 的缓存可以并存，切换时只加载各自通过校验的文件。
