"""Opt-in adapters that connect advisory AI costs to existing fleet algorithms.

Use ``AIEnhancedCoordinator`` in place of ``FleetCoordinator`` and
``AIEnhancedTaskAuction`` in place of ``TaskAuction`` when enabling this
optional extension. The stock classes and default fleet behavior stay intact.
"""
from math import cos, sin
from typing import Callable, Optional, Set

from . import astar
from .coordinator import FleetCoordinator
from .edge_ai import CongestionForecaster, OnlineTaskValuator
from .tasks import BATTERY_DERATE_PCT, TaskAuction


class AIEnhancedCoordinator(FleetCoordinator):
    """Drop-in coordinator adding decaying traffic heat to A* soft costs."""

    def __init__(self, *args, forecast_gain: float = 0.8,
                 heat_decay: float = 0.85, heat_horizon_s: float = 5.0,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.forecaster = CongestionForecaster(heat_decay, heat_horizon_s)
        self.forecast_gain = max(0.0, float(forecast_gain))
        self._last_heat_update: Optional[float] = None

    def replan(self, avoid_zones: Optional[Set[str]] = None,
               zone_penalty: float = 50.0,
               now: Optional[float] = None) -> bool:
        if self.goal_cell is None:
            self.path, self.intent = [], self.intent.__class__()
            return False

        now = self._last_now if now is None else now
        start = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        dt = (0.1 if self._last_heat_update is None
              else max(0.0, now - self._last_heat_update))
        robot_states = [self.state, *self.registry.alive().values()]
        predicted_cells = []
        sample_dt = 0.5
        sample_count = max(1, int(self.forecaster.horizon_s / sample_dt))
        for robot in robot_states:
            pose = robot.pose
            for step in range(sample_count + 1):
                t = min(self.forecaster.horizon_s, step * sample_dt)
                x = pose.x + robot.v * cos(pose.theta) * t
                y = pose.y + robot.v * sin(pose.theta) * t
                cell = self.grid.world_to_cell(x, y)
                if self.grid.in_bounds(cell):
                    predicted_cells.append(cell)
        self.forecaster.observe(predicted_cells, dt)
        self._last_heat_update = now

        blocked = self.grid.blocked_cells(now)
        blocked.update(self._peer_obstacle_cells())
        intents = [peer.intent for peer in self.registry.alive().values()]
        extra = astar.congestion_costs(intents, now)
        for cell, cost in self.forecaster.snapshot().items():
            extra[cell] = extra.get(cell, 0.0) + self.forecast_gain * cost

        for zone_id, penalty in self._zone_penalties.items():
            zone = self.grid.zones.get(zone_id)
            if zone:
                for cell in zone.cells:
                    extra[cell] = extra.get(cell, 0.0) + penalty
        for zone_id in (avoid_zones or set()):
            zone = self.grid.zones.get(zone_id)
            if zone:
                for cell in zone.cells:
                    extra[cell] = extra.get(cell, 0.0) + zone_penalty

        path = astar.astar(self.grid, start, self.goal_cell,
                           blocked=blocked, extra_cost=extra)
        if not path:
            self.log(f"AI REPLAN FAILED {start} -> {self.goal_cell}")
            self.path, self.intent = [], self.intent.__class__()
            return False
        self.path = path
        self.intent = astar.build_intent(self.grid, path, self.v_nominal, now)
        self.replan_count += 1
        return True


class AIEnhancedTaskAuction(TaskAuction):
    """Drop-in auction that discounts bids for predicted aisle congestion.

    ``task_congestion_cost`` receives the task and should return a finite,
    nonnegative cost. If no callback is supplied, the normal auction is used.
    """

    def __init__(self, *args,
                 task_congestion_cost: Optional[Callable] = None,
                 congestion_weight: float = 1.0,
                 valuator: Optional[OnlineTaskValuator] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.task_congestion_cost = task_congestion_cost
        self.congestion_weight = max(0.0, float(congestion_weight))
        self.valuator = valuator or OnlineTaskValuator()
        self._features_by_task = {}

    def compute_bid(self, task):
        bid = super().compute_bid(task)
        if bid is None:
            return bid
        congestion = 0.0
        if self.task_congestion_cost is not None:
            try:
                congestion = float(self.task_congestion_cost(task))
            except (TypeError, ValueError, OverflowError):
                return None
        if congestion < 0.0 or congestion != congestion or congestion == float('inf'):
            return None
        battery_risk = (
            max(0.0, BATTERY_DERATE_PCT - self._last_batt_after)
            / BATTERY_DERATE_PCT
        )
        urgent_idle = (task.priority >= 3 and
                       self.state_status in ('IDLE', 'CHARGING'))
        urgency_penalty = 0.0 if urgent_idle else 0.5
        features = [
            max(0.0, self._last_est) / 100.0,
            battery_risk,
            0.0,  # single-task robots; queued workload is normally zero
            urgency_penalty,
            self.congestion_weight * congestion,
        ]
        self._features_by_task[task.task_id] = features
        if len(self._features_by_task) > 1000:
            self._features_by_task.pop(next(iter(self._features_by_task)))
        predicted_cost = self.valuator.predict(features)
        if predicted_cost == float('inf'):
            return None
        return round(1.0 / (1.0 + predicted_cost), 6)

    def complete_current(self, now: float) -> None:
        task = self.my_task
        if task is not None:
            features = self._features_by_task.pop(task.task_id, None)
            elapsed = max(0.0, float(now) - float(task.created_at))
            if features is not None:
                self.valuator.observe(features, elapsed)
        super().complete_current(now)
