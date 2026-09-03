# Surface normal trend

## Purpose

Surface Fit / Normal Seed first fits a local surface trend from connected
SurfaceTarget samples and generates the seed normals. Its 3x3, 5x5, and 9x9
options select the discrete index-space support. Surface Normal then optionally
averages those seed normals; selecting None leaves the fitted seed field
unchanged. Neither stage moves samples or alters the reference mesh.

## Surface Fit / Normal Seed

The stage builds a connected-neighborhood surface trend in index space. The
3x3 mode uses a one-cell surface neighborhood and keeps up to nine connected
samples; the 5x5 mode uses a two-cell neighborhood and keeps up to 25 connected
samples; the 9x9 mode uses a four-cell neighborhood and keeps up to 81 connected
samples. The selected samples are fit in world space with a robust position
PCA, and the resulting normal is oriented by the SurfaceTarget seed normal.

The fit does not hard-remove candidates by angle; connectivity selects the
region, while robust residual weights reduce the influence of outliers.

When the source grid is available, the fitted direction is only sign-corrected
after the fit. A short probe on both sides of the local iso-surface crossing
compares the grid values; for a fog volume the lower-density side is outward,
while for a level set the higher-value side is outward. This changes only the
sign and cannot introduce a tangential tilt.

## Parameters

- Fit neighborhood selects the 3x3, 5x5, or 9x9 connected index-space region.
- Fit robust iterations repeat residual reweighting without changing the region.
- Trend neighborhood selects None, 3x3, or 5x5 for the optional second pass;
  None is the default.
- Trend strength blends the fitted seed field with its connected average.
- Robust iterations repeat the second-pass residual weighting.

## Stage behavior

Opening Surface Fit / Normal Seed shows the fitted seed normals. Opening Surface
Normal shows either those seed normals or the optional averaged field. Both
passes are computed only when the user presses Build/update, so parameter editing
does not block the render loop. Switching back to Surface Target restores the
raw seed preview without rebuilding the cache.
