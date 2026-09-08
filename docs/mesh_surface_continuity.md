# Mesh-derived propagation continuity

## Purpose

The orientation stage keeps the existing SurfaceTarget samples and the existing source mesh. It does not create a second smoothed mesh. Before normal propagation starts, it builds a cached continuity field from the already extracted source `SurfaceMesh`.

## Cached data

For each SurfaceTarget sample, the cache stores the nearest source-mesh vertex and a proximity factor. The source mesh adjacency is stored as compact CSR ranges, and each source vertex receives a local smoothness factor from the normal agreement of its one-ring mesh neighbors.

## Edge coefficient

For a candidate propagation edge `(a, b)`, the coefficient is the product of proximity, local smoothness, and a topology factor:

```
coefficient(a,b) = sqrt(proximity(a) * proximity(b))
                   * sqrt(smoothness(vertex(a)) * smoothness(vertex(b)))
                   * topology(vertex(a), vertex(b))
                   * tangent(vertex(a), vertex(b))
```

The topology factor is `1` for the same mesh vertex or a mesh edge, `0.6` when the two vertices share a one-ring neighbor, and `0` otherwise. This makes a spatially close but topologically unrelated surface branch unable to become a normal-propagation shortcut.

The tangent factor is `1 - max(abs(dot(d, n_a)), abs(dot(d, n_b)))^2`, clamped to `[0,1]`, where `d` is the target-point connection direction and `n_a/n_b` are the raw source-mesh vertex normals. A connection that travels through the mesh normal direction therefore receives little or no propagation weight while ordinary oblique surface links remain continuous.

## Propagation use

The coefficient is used continuously rather than as a new normal estimate:

- candidate ordering prefers a larger coefficient before equal-distance ties;
- coefficients below the orientation threshold are rejected;
- the selected parent supplies the sign for the candidate axis;
- transition propagation uses the same gate and already oriented Core normals.

The Core sign is now fixed by the selected parent. The candidate axis is flipped
only when its dot product with the parent normal is negative. A later neighborhood
vote is not allowed to reverse that parent-consistent result, which keeps a local
branch from changing orientation because another branch has more samples.

If the continuity cache is unavailable, the old behavior remains the fallback. A valid cache is built whenever the source mesh and SurfaceTarget cache are rebuilt.

## Scope of this version

The coefficient controls the expansion path and parent selection only. It does not
move SurfaceTarget points, replace the reconstruction mesh, or change the fitted
unoriented normal axes. This isolates the topology constraint so its effect can be
compared against the current pipeline.
