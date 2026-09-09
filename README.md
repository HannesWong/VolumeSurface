# VolumeSurface

该工程提供用于 OpenVDB 浮点 Fog Volume 的缓存友好 Brick Arena。模块设计与验证边界见 [Brick Arena 模块执行文档](docs/brick-arena-module.md)。

## 构建

默认配置使用以下本地依赖：

- OpenVDB 源码：`G:/downLoadProj/openvdb-master`
- OpenVDB 构建：`G:/downLoadProj/openvdb-master/build-vdb-from-slices`
- vcpkg 依赖：`G:/downLoadProj/vcpkg-master/installed/x64-windows`

路径可以通过对应的 `VOLUME_SURFACE_*` CMake Cache 变量覆盖。

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## 样例验证

```powershell
.\build\Release\vdb_brick_cache.exe G:\transformed\rightArm\rightArm.0224.vdb
```

工具默认读取 `density` Grid，构建 Brick Arena，并逐 Leaf 验证数值和 Active 状态。使用 `--output` 可以把当前缓冲区重新写入新的 VDB 文件。

## 诊断 Viewer

Filament 源码只需准备一次：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\prepare_filament.ps1
```

构建 Viewer：

```powershell
cmd.exe /d /c .\scripts\build_viewer_msvc_ninja.cmd
```

该脚本可从任意工作目录启动：自动定位工程根目录、选择可用的 Visual Studio 2022 x64 C++ 工具链、检查 Filament、配置 `build-viewer` 并只编译 `volume_surface_viewer`。如需使用非标准 MSVC 安装位置，可先将 `VOLUME_SURFACE_VCVARS64` 设为对应的 `vcvars64.bat` 绝对路径。

启动时默认读取右臂样例的 `density = 255` 等值面：

```powershell
.\build-viewer\volume_surface_viewer.exe
```

使用 `--input`、`--grid`、`--iso` 与 `--adaptivity` 可以覆盖默认参数。未提供 `--input` 时，Viewer 会读取 `viewer_inputs.jsonl`，先显示一个输入路径下拉框，确认后才加载并进入三维界面；当前列表将 `G:\images\male.vdb` 放在首位并默认选中。无界面模式会继续使用默认的 rightArm 样例，以便自动化测试不弹框。使用 `--inspect-only` 只验证读取、网格和三个方向的切片提取，不创建窗口；使用 `--headless-smoke` 在隐藏窗口中验证 Filament、纹理和 ImGui 资源后自动退出。完成的连续笔画会自动追加到 `brush_profile.jsonl`；使用 `--replay-brush-profile brush_profile.jsonl` 可以在隐藏 Viewer 中重放每次拟合并输出分段性能。Viewer 的启动阶段、异常和未处理异常会追加到 `viewer_error.jsonl`，文件与输入配置位于同一目录。具体格式和比较方法见 [笔刷性能记录与回放](docs/brush-performance-replay.md)，表面 BVH 的分组和增量边界见 [表面 BVH 笔刷加速](docs/brush-surface-bvh.md)，多个可保存权重场的工作流见 [权重场库](docs/weight-field-library.md)，流程节点和 Viewer 路由见 [流程控制设计](docs/workflow-control.md)，Surface Target 的点、法线和 BVH 预览见 [Surface Target 预览](docs/surface-target-preview.md)，局部法线种子和全局种子流程见 [法线定向流程](docs/normal-orientation-pipeline.md)，新的点驱动重建边界见 [表面重建重新设计](docs/surface-reconstruction-redesign.md)，输入选择和缓存切换见 [输入选择与缓存切换](docs/input-selection-and-cache-switching.md)。

当前面板提供 Reference、Result A、Result B 和 Result C 四个显示槽位。只有 Reference 已接入 Fog 等值面；另外三个槽位预留给后续平滑结果。详细边界见 [诊断 Viewer 执行文档](docs/diagnostic-viewer.md)。
