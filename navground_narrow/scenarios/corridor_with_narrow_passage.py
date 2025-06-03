from __future__ import annotations

from typing import cast

import numpy as np
from navground import sim

from ..modulations.narrow_corridor import NarrowModulation
from ..utils import NarrowPassage


def init_narrow_modulation(world: sim.World, seed: int | None = None) -> None:
    if hasattr(world, 'narrow'):
        narrow = cast('NarrowPassage', world.narrow)
    for agent in world.agents:
        if agent.behavior and agent.behavior.target.direction is not None:
            mod = NarrowModulation(narrow, world.random_generator, agent.id,
                                   agent.behavior.target.direction[0])
            agent.behavior.add_modulation(mod)


# TODO(Jerome) better to follow a point ... a direction make it go to the edge


class CorridorWithNarrowPassageScenario(sim.Scenario,
                                        name="CorridorWithNarrowPassage"
                                        ):  # type: ignore

    _length = 10.0
    _width = 1.0
    _narrow_width = 0.5
    _narrow_length = 2.0
    _biderectional = True

    def __init__(self,
                 length: float = 10.0,
                 width: float = 1.0,
                 agent_margin: float = 0.1,
                 add_safety_to_agent_margin: bool = True,
                 narrow_width: float = 0.5,
                 narrow_length: float = 2.0,
                 bidirectional: bool = True,
                 add_modulation: bool = False):
        super().__init__()
        self._add_modulation = add_modulation
        self._length = length
        self._width = width
        self._narrow_width = narrow_width
        self._narrow_length = narrow_length
        self._agent_margin = agent_margin
        self._add_safety_to_agent_margin = add_safety_to_agent_margin
        self._bidirectional = bidirectional

    def init_world(self, world: sim.World, seed: int | None = None) -> None:
        super().init_world(world, seed=seed)
        world.bounding_box = sim.BoundingBox(0, self.length, -self.width * 0.5,
                                             self.width * 0.5)
        dw = (self.width - self.narrow_width) * 0.5
        xb = self.length * 0.5 - self.narrow_length * 0.5
        xa = xb - dw
        xc = self.length * 0.5 + self.narrow_length * 0.5
        xd = xc + dw
        x0 = xc - self.length
        x1 = self.length + xb
        ps = np.array(
            ((x0, self.width * 0.5), (xa, self.width * 0.5),
             (xb, self.narrow_width * 0.5), (xc, self.narrow_width * 0.5),
             (xd, self.width * 0.5), (x1, self.width * 0.5)))
        for side in (-1, 1):
            for p1, p2 in zip(ps, ps[1:]):
                world.add_wall(sim.Wall(p1 * (1, side), p2 * (1, side)))

        def random_position():
            rng = world.random_generator
            x = rng.uniform(0.0, xa)
            y = rng.uniform(-self.width / 2, self.width / 2)
            if rng.integers(2):
                x = self.length - x
            return (x, y)

        for agent in world.agents:
            agent.position = random_position()
            agent.task = None

        world.set_lattice(0, (0.0, self.length))
        world.space_agents_apart(self.agent_margin,
                                 self.add_safety_to_agent_margin)

        world._prepare()
        for index, agent in enumerate(world.agents):
            orientation = 0.0
            direction = np.array((1.0, 0.0))
            if index % 2 and self.bidirectional:
                orientation = np.pi
                direction *= -1
            agent.orientation = orientation
            agent.controller.follow_direction(direction)
        world.narrow = NarrowPassage(  # type: ignore
            (np.asarray(
                (xa, self.width * 0.5)), np.asarray((xa, -self.width * 0.5))),
            (np.asarray(
                (xd, self.width * 0.5)), np.asarray((xd, -self.width * 0.5))))
        if self.add_modulation:
            init_narrow_modulation(world, seed=seed)

    @property
    @sim.register(False,
                  "Whether to add a Narrow behavior modulation to all agents")
    def add_modulation(self) -> bool:
        return self._add_modulation

    @add_modulation.setter
    def add_modulation(self, value: bool) -> None:
        self._add_modulation = value

    @property
    @sim.register(10.0, "The length of the corridor")
    def length(self) -> float:
        return self._length

    @length.setter
    def length(self, value: float) -> None:
        self._length = max(0, value)

    @property
    @sim.register(1.0, "The width of the corridor")
    def width(self) -> float:
        return self._width

    @width.setter
    def width(self, value: float) -> None:
        self._width = max(0, value)

    @property
    @sim.register(2.0, "The length of the narrow passage")
    def narrow_length(self) -> float:
        return self._narrow_length

    @narrow_length.setter
    def narrow_length(self, value: float) -> None:
        self._narrow_length = max(0, value)

    @property
    @sim.register(2.0, "The width of the narrow passage")
    def narrow_width(self) -> float:
        return self._narrow_width

    @narrow_width.setter
    def narrow_width(self, value: float) -> None:
        self._narrow_width = max(0, value)

    @property
    @sim.register(0.1, "The agent margin")
    def agent_margin(self) -> float:
        return self._agent_margin

    @agent_margin.setter
    def agent_margin(self, value: float) -> None:
        self._agent_margin = max(0, value)

    @property
    @sim.register(True, "Whether to add the safety margin to the agent margin")
    def add_safety_to_agent_margin(self) -> bool:
        return self._add_safety_to_agent_margin

    @add_safety_to_agent_margin.setter
    def add_safety_to_agent_margin(self, value: bool) -> None:
        self._add_safety_to_agent_margin = value

    @property
    @sim.register(True,
                  "Whether the traffic in the corridor in bi-directional")
    def bidirectional(self) -> bool:
        return self._bidirectional

    @bidirectional.setter
    def bidirectional(self, value: bool) -> None:
        self._bidirectional = value
