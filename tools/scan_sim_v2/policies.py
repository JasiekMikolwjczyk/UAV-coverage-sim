"""Confirmer dispatch policy interface (SPEC 6.2, 7.1).

A policy is a callable policy(state: PolicyState) -> list[Assignment]. It may assign each
free confirmer a suspicion (task) or an idle point. Confirmers it leaves out follow the
default idle behaviour of the simulator. Built-in policies go through the same interface.
"""
import importlib
import math
from dataclasses import dataclass

SCAN_SIZES = {"small": 0.5, "medium": 0.75, "full": 1.0}


@dataclass
class QueueItem:
    sid: int                 # object index
    x: float                 # estimated position
    y: float
    cov: tuple               # (var_x, var_y)
    t_arrival: float
    in_corridor: bool
    score: float             # screening score used by priority policy
    ahead: bool              # ahead of the vehicle in its lane


@dataclass
class ConfirmerView:
    cid: int
    x: float
    y: float
    available_t: float
    battery_percent: float
    free: bool


@dataclass
class VehicleView:
    x: float
    lane_idx: int
    lane_y: float
    direction: int
    blocked: bool


@dataclass
class PolicyState:
    t: float
    queue: list
    confirmers: list
    vehicle: VehicleView
    frontier_x: float
    legacy_ordering: bool = False


@dataclass
class Assignment:
    confirmer_id: int
    suspicion_id: int | None = None
    idle_point: tuple | None = None
    scan_size: str = "full"


def _free(state):
    return [c for c in state.confirmers if c.free]


def priority_key(item: QueueItem, legacy: bool):
    if legacy:
        return (-item.score, item.sid)          # v1 heap order
    return (-item.score, item.t_arrival, item.sid)


def priority_policy(state: PolicyState):
    remaining = sorted(state.queue, key=lambda it: priority_key(it, state.legacy_ordering))
    out = []
    for conf in _free(state):
        if not remaining:
            break
        item = remaining.pop(0)
        out.append(Assignment(conf.cid, item.sid))
    return out


def _nearest(conf, items):
    return min(items, key=lambda it: (math.hypot(it.x - conf.x, it.y - conf.y), it.t_arrival, it.sid))


def nearest_policy(state: PolicyState):
    remaining = list(state.queue)
    out = []
    for conf in _free(state):
        if not remaining:
            break
        item = _nearest(conf, remaining)
        remaining.remove(item)
        out.append(Assignment(conf.cid, item.sid))
    return out


def corridor_first_policy(state: PolicyState):
    remaining = list(state.queue)
    out = []
    for conf in _free(state):
        if not remaining:
            break
        urgent = [it for it in remaining if it.in_corridor and it.ahead]
        item = _nearest(conf, urgent if urgent else remaining)
        remaining.remove(item)
        out.append(Assignment(conf.cid, item.sid))
    return out


BUILTIN = {
    "priority": priority_policy,
    "nearest": nearest_policy,
    "corridor_first": corridor_first_policy,
}


def resolve_policy(name):
    """Built-in name, or dotted path 'pkg.mod.func' / 'pkg.mod:func', or a callable."""
    if callable(name):
        return name
    if name in BUILTIN:
        return BUILTIN[name]
    if ":" in name:
        mod_name, attr = name.split(":", 1)
    else:
        mod_name, _, attr = name.rpartition(".")
    if not mod_name:
        raise ValueError(f"unknown confirmer_policy {name!r}")
    mod = importlib.import_module(mod_name)
    fn = getattr(mod, attr)
    if not callable(fn):
        raise ValueError(f"confirmer_policy {name!r} is not callable")
    return fn
