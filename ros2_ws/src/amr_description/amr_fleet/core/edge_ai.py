"""Optional, dependency-free advisory features for EdgeSwarm.

These functions use only standard Python and never affect safety decisions.
They can be replaced by a learned model while retaining deterministic fallbacks.
"""

from collections import defaultdict
from math import exp, isfinite
from typing import Dict, Iterable, Mapping, Tuple

Cell = Tuple[int, int]


class CongestionForecaster:
    """Exponential occupancy forecast over grid cells from observed traffic."""

    def __init__(self, decay: float = 0.85, horizon_s: float = 5.0):
        if not 0.0 < decay < 1.0:
            raise ValueError('decay must be between zero and one')
        if horizon_s <= 0:
            raise ValueError('horizon_s must be positive')
        self.decay = decay
        self.horizon_s = horizon_s
        self._heat: Dict[Cell, float] = defaultdict(float)
        self._flow: Dict[Cell, float] = defaultdict(float)

    def observe(self, robot_cells: Iterable[Cell], dt_s: float) -> None:
        """Update from current robot positions; dt_s is capped to avoid spikes."""
        dt_s = max(0.0, min(float(dt_s), self.horizon_s))
        attenuation = self.decay ** max(dt_s, 0.05)
        for cell in list(self._heat):
            self._heat[cell] *= attenuation
            self._flow[cell] *= attenuation
        for cell in robot_cells:
            self._heat[cell] = min(1.0, self._heat[cell] + 0.25)
            self._flow[cell] = min(1.0, self._flow[cell] + 0.1)

    def cost(self, cell: Cell) -> float:
        """Return a nonnegative additive A* cost, bounded to [0, 4]."""
        heat = min(1.0, max(0.0, self._heat.get(cell, 0.0)))
        flow = min(1.0, max(0.0, self._flow.get(cell, 0.0)))
        return min(4.0, 3.0 * heat + flow)

    def snapshot(self) -> Mapping[Cell, float]:
        return {cell: self.cost(cell) for cell in self._heat}


def advisory_task_cost(distance_m: float, battery_pct: float,
                       congestion_cost: float = 0.0,
                       energy_per_m_pct: float = 0.35) -> float:
    """Lower is better; estimates travel, low-battery penalty and congestion.

    Invalid telemetry returns infinity so callers can decline a bid instead of
    making an optimistic assignment from corrupt values.
    """
    values = (distance_m, battery_pct, congestion_cost, energy_per_m_pct)
    if not all(isfinite(float(v)) for v in values):
        return float('inf')
    if distance_m < 0 or energy_per_m_pct < 0:
        return float('inf')
    battery = min(100.0, max(0.0, float(battery_pct)))
    reserve = distance_m * energy_per_m_pct
    if battery - reserve < 10.0:
        return float('inf')
    scarcity = 1.0 + max(0.0, 30.0 - battery) / 20.0
    return max(0.0, distance_m) * scarcity + max(0.0, congestion_cost) + reserve * 0.5


class OnlineTaskValuator:
    """Tiny online linear regressor for end-to-end task time/cost.

    Features are already normalized by the caller. Weights start at a
    transparent rule-based prior and adapt with clipped SGD after outcomes.
    There is no ML framework, model download, or inference process dependency.
    """

    def __init__(self, initial_weights=(100.0, 40.0, 20.0, 15.0, 1.0),
                 learning_rate: float = 0.02,
                 max_weight: float = 1000.0):
        self.weights = [float(value) for value in initial_weights]
        if not self.weights or any(not isfinite(value) or value < 0.0
                                   for value in self.weights):
            raise ValueError('initial_weights must be finite and nonnegative')
        self.learning_rate = max(0.0, min(0.1, float(learning_rate)))
        self.max_weight = max(1.0, float(max_weight))
        self.samples = 0
        self.last_absolute_error = 0.0

    def predict(self, features: Iterable[float]) -> float:
        values = [float(value) for value in features]
        if len(values) != len(self.weights) or not all(isfinite(v) for v in values):
            return float('inf')
        return max(0.0, sum(w * max(0.0, x)
                             for w, x in zip(self.weights, values)))

    def observe(self, features: Iterable[float], observed_cost: float) -> bool:
        values = [max(0.0, float(value)) for value in features]
        target = float(observed_cost)
        if (len(values) != len(self.weights) or
                not all(isfinite(v) for v in values) or
                not isfinite(target) or target < 0.0):
            return False
        prediction = self.predict(values)
        error = max(-300.0, min(300.0, target - prediction))
        norm = 1.0 + sum(value * value for value in values)
        step = self.learning_rate * error / norm
        for i, value in enumerate(values):
            self.weights[i] = min(
                self.max_weight,
                max(0.0, self.weights[i] + step * value),
            )
        self.samples += 1
        self.last_absolute_error = abs(error)
        return True
