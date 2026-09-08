# Local flip-point propagation

## Role

The global orientation seed remains the required global sign anchor. A point
added in the panel after Fit is a local flip point: its normal is forced to
reverse relative to the current oriented field. It is not a second global
orientation seed.

## Chain rule

The propagation preview builds a directed tree before replay:

1. The selected point is inserted with depth `0` and its post-flip direction
   is `-currentNormal`.
2. Only children in the global orientation trace are eligible; a neighbour is
   considered only when its recorded parent is the current flipped point.
3. A child whose current direction is strongly opposite is another link in
   the flip chain. It is added at the next depth and its post-flip direction
   is `-neighbourCurrentNormal`.
4. Parent nodes and lateral branches are never revisited. A child already
   aligned with the flipped parent is a hard boundary and is not crossed.
   Weak or invalid links are also boundaries.

The chain therefore supports multiple consecutive reversal points while
preventing a correction from crossing a region that is already consistent.
Parent links and depths are retained in the preview for diagnostics and future
chain visualization.

## Commit and replay

Add selected Core writes the point to the local JSONL immediately, enables it,
and replays the flip from the global baseline, so the list survives an
application restart with the same active behavior. Replay selected flips
exactly the Core samples in the preview's affected set, then re-orients
Transition samples from the majority of adjacent flipped Core samples.
Accepted points are replayed in list order from the immutable global
orientation result. Existing cache files remain readable; newly written point
records use the `flip_point` JSONL kind and include the accepted state.

## Local panel diagnostics

The Local Flip Points panel can inspect the global Seed propagation parent
chain. Picking a Core point only selects and visualizes that chain; the
`Add selected Core as flip point` action writes and applies the local flip
point immediately. The same panel can display the current working normal
vectors and the fitted axis at the selected Core point.
