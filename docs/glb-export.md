# Reconstruction GLB Export

`Review / Export` only writes `Result A`, which is the reconstructed mesh produced by the Surface Reconstruction stage. The source mesh, excluded yellow component, result slots B/C, debug points, and brush overlays are not included.

## File contents

The writer emits a minimal glTF 2.0 binary file with one triangle primitive. The primitive always contains `POSITION` and 32-bit triangle indices. `NORMAL` and its binary buffer are optional and are controlled by the Export panel; the checkbox is disabled by default. No material, texture, color, or viewer lighting data is written.

Vertex coordinates are copied from the reconstructed `SurfaceMesh` in its VDB/world coordinate system. Viewer centering and display scaling are not exported. The GLB therefore preserves the reconstruction coordinates exactly; the physical unit remains the unit used by the VDB transform.

## Viewer behavior

The panel defaults the export folder to the selected VDB's parent directory. The folder button opens a Windows folder picker, the name field accepts a file name, and `.glb` is appended when omitted. Existing files require an explicit overwrite confirmation.
