# Surface normal field

## Purpose

This stage regularizes the normals attached to the existing SurfaceTarget
samples. It does not move the samples or alter the reference mesh.

## First iteration

The stage builds a physical-distance, confidence-weighted normal field from the
Core samples. Distance, normal-angle, and same-sheet weights are continuous. The
result is displayed through the existing point-cloud and normal-line preview.

Reconstruction remains unchanged until the filtered field has been inspected.

## Parameters

- Smoothing radius controls the physical Core neighborhood in millimeters.
- Strength blends the original normal with the weighted neighborhood mean.
- Iterations applies the same Jacobi-style update repeatedly.
- Angle sigma controls how strongly normal disagreement reduces a neighbor's
  contribution.
- Sheet thickness controls separation between nearby but non-coplanar sheets.
- Maximum neighbors bounds the local work for dense regions.

## Stage behavior

Opening Normal Field shows the SurfaceTarget samples with either raw or filtered
normals. The filtered result is computed only when the user presses Build/update,
so parameter editing does not block the render loop. Switching back to Surface
Target restores the raw normal preview without rebuilding the cache.
