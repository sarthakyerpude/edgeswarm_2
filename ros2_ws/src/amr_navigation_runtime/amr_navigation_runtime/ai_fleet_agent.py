"""Opt-in launcher entry point that runs the standard fleet node with AI adapters.

The stock fleet node remains untouched. This module swaps its constructor
references before main() instantiates FleetAgentNode, enabling the optional
traffic-cost coordinator and congestion-aware auction.
"""
from amr_fleet.core import astar
from amr_fleet.core.edge_ai_integration import (
    AIEnhancedCoordinator,
    AIEnhancedTaskAuction,
)
from amr_fleet.nodes import fleet_agent_node


class _WiredTaskAuction(AIEnhancedTaskAuction):
    def __init__(self, *args, **kwargs):
        path_length = kwargs.get('path_length_m')
        coordinator = getattr(path_length, '__self__', None)
        if coordinator is None or not hasattr(coordinator, 'forecaster'):
            raise RuntimeError('AI auction requires AIEnhancedCoordinator')

        def task_congestion_cost(task):
            extra = {
                cell: coordinator.forecast_gain * cost
                for cell, cost in coordinator.forecaster.snapshot().items()
            }
            now = coordinator._last_now
            blocked = coordinator.grid.blocked_cells(now)
            start = coordinator.grid.world_to_cell(
                coordinator.state.pose.x, coordinator.state.pose.y
            )
            start_to_pickup = astar.astar(
                coordinator.grid, start, task.pickup,
                blocked=blocked, extra_cost=extra,
            )
            pickup_to_dropoff = astar.astar(
                coordinator.grid, task.pickup, task.dropoff,
                blocked=blocked, extra_cost=extra,
            )
            if not start_to_pickup or not pickup_to_dropoff:
                return float('inf')
            return sum(extra.get(cell, 0.0) for cell in start_to_pickup[1:]) + sum(
                extra.get(cell, 0.0) for cell in pickup_to_dropoff[1:]
            )

        kwargs['task_congestion_cost'] = task_congestion_cost
        super().__init__(*args, **kwargs)


def main(args=None):
    fleet_agent_node.FleetCoordinator = AIEnhancedCoordinator
    fleet_agent_node.TaskAuction = _WiredTaskAuction
    fleet_agent_node.main(args=args)
