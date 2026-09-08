# Surface normal fit and local flips

## Purpose

Surface Fit / Normal Seed first fits a local surface trend from connected
SurfaceTarget samples and generates unoriented axes. Its 3x3, 5x5, and 9x9
options select the discrete index-space support. The panel after Fit only
manages explicit local flip points; smoothing belongs to a later dedicated
stage. Neither stage moves samples or alters the reference mesh.

## Surface Fit / Normal Seed

The stage builds a connected-neighborhood surface trend in index space. The
3x3 mode uses a one-cell surface neighborhood and keeps up to nine connected
samples; the 5x5 mode uses a two-cell neighborhood and keeps up to 25 connected
samples; the 9x9 mode uses a four-cell neighborhood and keeps up to 81 connected
samples. The selected samples are fit in world space with a robust position
PCA, and the resulting normal is retained as an unoriented axis.

The fit does not hard-remove candidates by angle; connectivity selects the
region, while robust residual weights reduce the influence of outliers.

The initial orientation seed is applied after PCA fitting and establishes the
main surface direction. Local flip points are handled in the panel after Fit,
where their affected regions are previewed before an accepted flip point is
committed. The
full stage ordering is specified in [Normal orientation pipeline](normal-orientation-pipeline.md).

## Parameters

- Fit neighborhood selects the 3x3, 5x5, or 9x9 connected index-space region.
- Fit robust iterations repeat residual reweighting without changing the region.
## Stage behavior

Opening Surface Fit / Normal Seed shows the fitted axes and applies the global
seed. Opening Local Flip Points exposes local flip-point preview and commit.
Smoothing is intentionally not started from this panel, so changing local
flip-point parameters only recomputes the bounded preview chain.
