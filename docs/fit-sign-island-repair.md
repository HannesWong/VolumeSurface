# Fit sign-island repair

> Retired: automatic sign-island repair is no longer part of the runtime
> pipeline. This document is kept as historical context for the removed
> strategy; local corrections now use explicit seeds in the panel after Fit.

## Purpose

The repair pass removes small local sign inversions that remain after primary
seed orientation.
It changes only `n` to `-n`; it does not change fitted axes, samples, or the
absolute outward orientation of a connected surface.

## Data flow

`Surface Fit` produces unsigned axes first. The primary seed orients those
axes. The repair pass then evaluates every valid Core-to-Core surface adjacency
edge in that seed-oriented field, derives edge and point confidence from the
complete edge set, groups high-confidence same-direction regions, and flips
accepted small opposing regions. `Surface Normal` consumes the repaired,
oriented field.

## Edge evaluation

Only Core samples participate. A candidate is a 26-neighbor pair that passes
the existing surface-continuation test. Each undirected edge stores the sample
indices; every evaluation derives its signed dot from the current repaired
field, so later repair passes see earlier flips.

The signed dot is not a binary decision. Its magnitude contributes to axis
reliability, while its sign contributes either same-direction or
opposition support. Point reliability is derived from all incident edges, so a
single unstable edge cannot create a repair candidate by itself.

## Region repair

High-confidence same-direction edges create regions. Every high-confidence
opposition edge is then accumulated between its two regions. A region is a
repair candidate only when it is physically small, its dominant opposing
boundary reaches a larger host region, and flipping the region reduces the
weighted opposing boundary support.

All accepted regions are evaluated against the same immutable primary-seed
field and committed atomically. The repair pass does not cascade through
newly-flipped regions, so a decision made for one island cannot manufacture a
second candidate in the same build.

## Scope and safeguards

The repair pass does not choose global inward or outward orientation; the
primary seed owns that decision. It leaves similarly sized regions,
disconnected fragments, and regions with ambiguous multi-host boundaries
unchanged. Transition samples are reoriented only from the repaired Core field
after Core repair completes.

The Fit stage keeps the primary-seed and repaired fields for comparison. The
repair report records candidate and accepted island counts, flipped sample
count, physical area, and weighted opposing support before and after repair.
Candidate regions additionally require a minimum number of opposing boundary
edges and a positive repair-energy margin. When orientation propagation trace
data is available, shallow seed neighborhoods are protected from automatic
flipping. Parent links that cross a candidate boundary remain part of the
boundary's agreeing support, so they penalize but do not unconditionally forbid
an otherwise well-supported region repair.

The controls are intentionally separate: `minimumRegionConnectivity` controls
weak sign-invariant region growth, `minimumAlignment` cuts strong opposing
edges and supplies conflict support, `minimumOpposingBoundaryEdges` rejects
single-edge accidents, and `minimumRepairEnergyMargin` requires a clear gain
from flipping. `maximumPasses` is no longer used for cascading edits.

## Verification

Unit tests cover an isolated reversed Core island, a similarly sized opposing
pair that must remain unchanged, a disconnected small component that must
remain unchanged, and world-space area behavior on anisotropic sample spacing.
