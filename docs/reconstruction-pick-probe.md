# Reconstruction pick probe

## Purpose

Reconstruction provides an armed pick action for one visible mesh point. The
probe records both the picked mesh position and a nearby position projected to
the VDB iso-surface so that a problematic triangle can be reported in a form
that can be pasted back for local analysis.

## Coordinate contract

The copied report contains scene coordinates for renderer diagnostics, world
coordinates in millimeters for external discussion, floating-point VDB index
coordinates, and the nearest cached SurfaceTarget sample coordinate. World
coordinates are not replaced by scene coordinates because the viewer applies a
center and display-scale transform before rendering.

## Projection

The probe starts at the picked mesh position and performs a bounded Newton
projection against the source grid's interpolated density field. The result
contains the projected iso position, residual, sampled density, gradient
normal, displacement from the picked mesh, and iteration count. If projection
does not converge, the original picked position remains available and the
report marks the projection as invalid.

## Clipboard format

`Copy VDB location` writes JSONL with one record for the source and pick, one
record for the VDB projection, and one record for the nearest SurfaceTarget
sample. The format is intentionally compact and deterministic so a report can
be pasted into a later local reconstruction comparison.
