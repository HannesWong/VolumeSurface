# Normal orientation pipeline

## Runtime status

Automatic sign-island repair is retired from the runtime pipeline. The former
repair document is retained only as historical context; no Fit, Local Flip
Points, smoothing, or reconstruction path may call it.

## Purpose

The primary orientation seed establishes the global sign of the fitted normal
field and remains maintained in the Surface Fit panel. It is a required
computation step, not a preview option. Automatic
sign-island repair is retired; explicit local flip points are the only
correction mechanism after primary seed propagation. Their chain rule is
defined in `local-flip-point-propagation.md`.

## Processing order

1. Surface Fit estimates an unsigned local PCA axis at every SurfaceTarget
   sample.
2. The saved primary seed orients those axes across the connected surface.
3. The panel after Fit previews one local flip point's maximum affected region using
   transparent points, without mutating the oriented field.
4. The user accepts or cancels that local flip point. Accepted local flip points are
   replayed in list order from the immutable global-seed result.
5. Later smoothing and reconstruction stages consume the latest oriented result
   after the accepted local flip points.

## Preview contract

After primary seed propagation has completed, the Fit preview shows the
seed-oriented field. Local flip-point preview and commit controls live in the
panel after Fit, not in the Fit panel.

## Field roles

The unsigned fitted axes are retained for fit inspection. A separate primary
orientation field records the result immediately after the global seed. The
working oriented field records the result after replaying accepted local
flip points. Keeping these fields separate prevents a UI preview from becoming an
implicit algorithmic branch.
