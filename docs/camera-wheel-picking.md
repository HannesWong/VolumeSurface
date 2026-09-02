# Camera wheel picking

## Goal

Use the current mouse ray to pick the nearest visible reference or reconstructed
surface before applying a wheel event. Scale the dolly step by the projected
camera-to-hit distance, keep a 1 mm minimum requested step, and preserve a
separate physical clearance margin when moving toward the surface.

The main camera clip planes are configured in physical units: near = 0.1 mm and
far = 10 m. They are converted through the current display scale before being
passed to Filament.

## Design

- Build a compact flat triangle BVH for each mesh slot that can be picked.
- Unproject the mouse ray with Filament's finite culling projection; the rendering
  projection has an infinite far plane and cannot safely provide a finite far point.
- Keep this mesh BVH separate from the VDB leaf hierarchy: the latter is a
  coarse voxel query structure and cannot provide an exact triangle hit alone.
- Store BVH vertices in Filament scene coordinates so ray tests use the same
  coordinate system as the camera.
- Let `ViewerDisplayManager` adapt wheel events before `FilamentApp2` forwards
  them to its orbit manipulator.
- Pick only visible reference and result slots; choose the nearest positive
  intersection when both are visible.
- Convert the minimum 1 mm step and clearance margin to scene units through the
  current display scale.
- Preserve the existing wheel-speed control as a multiplier for the adaptive
  step, using 12x as the default reference.
- Fall back to the existing fixed wheel multiplier when the pointer is over UI,
  the camera is not initialized, or no mesh is hit.

## Update points

- Rebuild or clear a slot BVH whenever its mesh is created, replaced, or
  destroyed.
- Keep the latest hit point and distance in viewer state for diagnostics.
- Do not change Filament's bundled camera manipulator or renderable visibility
  rules.

## Verification

- Build core and viewer with the MSVC Ninja scripts.
- Run the existing six core tests.
- Run headless smoke and inspect-only startup checks.
- Verify that changing reference/result visibility changes the picked surface
  without affecting other workflow presentation rules.
