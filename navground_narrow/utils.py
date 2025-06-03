from __future__ import annotations

import numpy as np
from navground import core
from shapely import geometry as g


class NarrowPassage:

    def __init__(self, door_1: tuple[core.Vector2, core.Vector2],
                 door_2: tuple[core.Vector2, core.Vector2]):
        d1 = np.asarray(door_1)
        d2 = np.asarray(door_2)
        delta = np.mean(d2, axis=0) - np.mean(d1, axis=0)
        self.length = float(np.linalg.norm(delta))
        self.e = delta / self.length
        n = np.array((self.e[1], -self.e[0]))
        if np.dot(d1[1] - d1[0], n) < 0:
            door_1 = door_1[::-1]
        if np.dot(d2[1] - d2[0], n) > 0:
            door_2 = door_2[::-1]
        self.area = g.Polygon(list(door_1) + list(door_2))
        self.doors = (core.LineSegment(*door_1), core.LineSegment(*door_2))

    def contains(self, position: core.Vector2) -> bool:
        return self.area.contains(g.Point(position))

    def time_to_enter(self,
                      position: core.Vector2,
                      velocity: core.Vector2,
                      index: int = -1) -> tuple[float, int]:
        for i, d in enumerate(self.doors):
            if index >= 0 and i != index:
                continue
            t = d.distance_along(position, velocity, 1)
            if t >= 0:
                return t, i
        return -1, index

    def time_to_exit(self,
                     position: core.Vector2,
                     velocity: core.Vector2,
                     index: int = -1) -> tuple[float, int]:
        if not self.contains(position):
            return -1, index
        for i, d in enumerate(self.doors):
            if index >= 0 and i != index:
                continue
            t = d.distance_along(position, velocity, -1)
            # print('T', t, d, position, velocity)
            if t >= 0:
                return t, i
        return -1, index

    def is_exiting(self,
                   position: core.Vector2,
                   orientation: float,
                   velocity: core.Vector2,
                   index: int = -1) -> bool:
        if not self.contains(position):
            return False
        if index < 0:
            return True
        d = self.doors[index]
        if not np.any(velocity):
            velocity = np.array((np.cos(orientation), np.sin(orientation)))

        t = d.distance_along(position, velocity, -1)
        return t > 0

    def enter_position(self, index: int) -> core.Vector2:
        p = 0.5 * (self.doors[index].p1 + self.doors[index].p2)
        return p
