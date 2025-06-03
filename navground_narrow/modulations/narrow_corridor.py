from __future__ import annotations

import numpy as np
from navground import core

from ..utils import NarrowPassage


def find_conflict(
        narrow: NarrowPassage, position: core.Vector2, velocity: core.Vector2,
        radius: float, margin: float, horizon: float,
        neighbors: list[core.Neighbor]) -> tuple[core.Neighbor, int] | None:

    # are we inside?
    if narrow.contains(position):
        # print('we inside')
        return None

    # will we enter?
    t, index = narrow.time_to_enter(position, velocity)
    if t < 0:
        # print('we not entering')
        return None
    # soon enough?
    speed = float(np.linalg.norm(velocity))
    distance = t * speed
    if distance > horizon:
        # print('we entering too far', distance, t, speed)
        return None

    # are there agents inside moving towards us?

    # inside_neighbors = [n for n in neighbors if narrow.time_to_exit(n.position, n.velocity, index)[0] > 0]
    inside_neighbors = [
        n for n in neighbors if narrow.contains(n.position)
        and narrow.time_to_exit(n.position, n.velocity, 1 - index)[0] < 0
    ]

    # if yes, then return the one with the largest radius
    if inside_neighbors:
        return sorted(inside_neighbors, key=lambda n: n.radius,
                      reverse=True)[0], index

    # are there neighbor that are currenly outside but would reach the half way of narrow before us?
    outside_neighbor = []
    # time for us to reach half way
    t += (narrow.length * 0.5 - radius) / speed
    for n in neighbors:
        tn, _ = narrow.time_to_enter(n.position, n.velocity, 1 - index)
        if tn < 0 or tn > t:
            continue
        speedn = float(np.linalg.norm(n.velocity))
        # time for them to reach half way
        tn += (narrow.length * 0.5 - n.radius - margin) / speedn
        if tn < t:
            outside_neighbor.append(n)

    # if yes, then return the one with the largest radius
    if outside_neighbor:
        return sorted(outside_neighbor, key=lambda n: n.radius,
                      reverse=True)[0], index

    # print('no conflicts')
    return None


class NarrowModulation(core.BehaviorModulation, name="Narrow"):

    def __init__(self,
                 narrow: NarrowPassage | None = None,
                 rg: np.random.Generator | None = None,
                 i: int = -1,
                 direction: int = 1):
        super().__init__()
        self.narrow = narrow
        self.extra_line_obstacles: list[core.LineSegment] = []
        self.i = i
        self.d = direction
        self.rg = rg

    def post(self, behavior: core.Behavior, time_step: float,
             cmd: core.Twist2) -> core.Twist2:
        return cmd

    def pre(self, behavior: core.Behavior, time_step: float) -> None:
        if self.narrow and behavior.target.direction is not None and isinstance(
                behavior.environment_state, core.GeometricState):
            ns = behavior.environment_state.neighbors
            v = behavior.optimal_speed * behavior.target.direction
            c = find_conflict(self.narrow, behavior.position, v,
                              behavior.radius, behavior.safety_margin,
                              behavior.horizon,
                              [n for n in ns if n.id != self.i])
            if c:
                n, index = c
                assert (n.id != self.i), (behavior.position, n)
                line_obs = self.narrow.doors[index]
                if line_obs not in self.extra_line_obstacles:
                    # behavior.environment_state.line_obstacles.append(line_obs)

                    # print('appended line', line_obs, behavior.environment_state.line_obstacles)
                    self.extra_line_obstacles.append(line_obs)
                    behavior.environment_state.line_obstacles = behavior.environment_state.line_obstacles + [
                        line_obs
                    ]

                # e = np.random.rand(2) * 0.1
                if self.rg is not None:
                    e = self.rg.uniform(size=2, high=0.1)
                else:
                    e = np.zeros(2)
                q = self.narrow.enter_position(index)
                m = core.Neighbor(q + e, n.radius, n.velocity, id=n.id)
                ns.append(m)
                behavior.environment_state.neighbors = ns

            else:
                if self.extra_line_obstacles:
                    lns = behavior.environment_state.line_obstacles
                    for x in self.extra_line_obstacles:
                        # print('try to remove line', x, ns[-1], x == ns[-1])
                        lns.pop(-1)
                        # behavior.environment_state.line_obstacles.remove(x)
                    self.extra_line_obstacles = []
                    behavior.environment_state.line_obstacles = lns
            for n in behavior.environment_state.neighbors:
                # (n.position[0] - q[0]) * d < 0
                if n.id == self.i and (n.position[0] -
                                       behavior.position[0]) * self.d > 0:
                    n.radius += 0.9
                    # print(i, d, behavior.position, n)
