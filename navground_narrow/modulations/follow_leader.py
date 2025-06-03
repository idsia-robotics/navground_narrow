from __future__ import annotations

import numpy as np
from navground import core


def split_flows(
        ns: list[core.Neighbor], direction: np.ndarray
) -> tuple[list[core.Neighbor], list[core.Neighbor]]:
    same_flow_ns = []
    opposing_flow_ns = []
    for n in ns:
        v = n.velocity.dot(direction)
        if v > 0:
            same_flow_ns.append(n)
        elif v < 0:
            opposing_flow_ns.append(n)
    return same_flow_ns, opposing_flow_ns


class FollowLeaderModulation(core.BehaviorModulation, name="FollowLeader"):

    def __init__(self):
        super().__init__()

    def post(self, behavior: core.Behavior, time_step: float,
             cmd: core.Twist2) -> core.Twist2:
        behavior.optimal_speed = self.v
        return cmd

    def pre(self, behavior: core.Behavior, time_step: float) -> None:
        if not isinstance(behavior.environment_state, core.GeometricState):
            return
        self.v = behavior.optimal_speed
        ns = behavior.environment_state.neighbors
        d = behavior.target.direction
        if d is None:
            return
        in_front_ns = [
            n for n in ns if d.dot(n.position - behavior.position) > 0
        ]
        same_flow_ns, opposing_flow_ns = split_flows(in_front_ns, d)
        if not same_flow_ns:
            return

        opposing_flow_ns = sorted(opposing_flow_ns,
                                  key=lambda n: d.dot(n.position))
        if not opposing_flow_ns:
            return

        m = opposing_flow_ns[0]

        same_flow_ns = sorted(same_flow_ns, key=lambda n: d.dot(n.position))

        if not same_flow_ns:
            return
        n = same_flow_ns[0]

        delta = d.dot(n.position -
                      behavior.position) - n.radius - behavior.radius
        dx = (m.radius * 2 - delta)

        behavior.optimal_speed = max(0,
                                     min(self.v,
                                         d.dot(n.velocity) - dx / 0.5))
