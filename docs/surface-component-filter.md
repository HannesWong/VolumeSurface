# Surface component filtering

Source VDB extraction can produce several disconnected surface components. The largest edge-connected triangle component is retained as the primary source mesh. Every other component is compacted into an excluded mesh and is shown in yellow only during the Source stage.

The SurfaceTarget cache applies the same primary-component rule to Core samples and keeps only transition samples attached to the retained Core component. Normal fitting, local flip points, brush weights, and reconstruction consume this filtered cache. The excluded mesh and excluded samples never enter those stages.

Reconstruction filters its generated mesh before exposing the result slot. A source-topology fallback uses the same filter. Export should therefore consume the primary mesh/result slots directly and must not export the excluded source mesh.

At runtime the excluded surface is also rasterized into a sparse three-voxel narrow-band `BoolGrid` using the source VDB transform. This mask is kept in the current `DocumentSession` for later VDB-based filters; it is intentionally memory-only and does not modify the source VDB.
