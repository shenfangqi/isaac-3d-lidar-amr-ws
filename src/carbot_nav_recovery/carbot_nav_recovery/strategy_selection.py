"""Deterministic recovery options for review; never authorizes motion."""

from dataclasses import dataclass

from .swept_footprint import rank_rotation_candidates


@dataclass(frozen=True)
class RecoveryOption:
    strategy: str
    candidate: object
    motion_eligible: bool = False


def recovery_options(rotations=(), trace_retreats=()):
    """Prefer departure-checked rotations, then nearest historical refuge.

    A caller can display and further validate these options. Trace retreat
    still needs a fresh departure plan to the original goal, in addition to
    online clearance, ownership, stop and budget gates before execution.
    """
    options = [RecoveryOption('ROTATE', candidate)
               for candidate in rank_rotation_candidates(rotations)
               if candidate.safe]
    options.extend(
        RecoveryOption('RETRACE_MEASURED_PATH', candidate)
        for candidate in sorted(trace_retreats,
                                key=lambda item: item.distance_m)
        if candidate.geometry_clear)
    return tuple(options)
