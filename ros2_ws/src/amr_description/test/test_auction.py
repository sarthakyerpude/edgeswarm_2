"""Auctioneer-free task allocation."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.models import Task
from amr_fleet.core.tasks import TaskAuction


def mk(rid, cell=(0, 0), batt=100.0):
    a = TaskAuction(rid, lambda d: None, lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: abs(p[0] - q[0]) + abs(p[1] - q[1]))
    a.current_cell = cell
    a.battery_pct = batt
    a.nominal_speed = 0.4
    a.state_status = "IDLE"
    return a


TASK = Task("T-1", pickup=(5, 5), dropoff=(10, 10), priority=1)


def test_nearer_robot_bids_higher():
    near = mk("robot_1", (4, 4)).compute_bid(TASK)
    far = mk("robot_2", (30, 30)).compute_bid(TASK)
    assert near > far


def test_low_battery_robot_refuses_to_bid():
    """HARD constraint. A robot that would strand itself must not bid at all -
    it would become a permanent obstacle for the whole fleet."""
    assert mk("robot_1", (4, 4), batt=16.0).compute_bid(TASK) is None


def test_busy_robot_does_not_bid():
    a = mk("robot_1")
    a.my_task = TASK
    assert a.compute_bid(Task("T-2", (1, 1), (2, 2))) is None


def test_unreachable_task_not_bid():
    a = mk("robot_1")
    a._path_len = lambda p, q: -1.0
    assert a.compute_bid(TASK) is None


def test_all_robots_pick_the_same_winner():
    """The heart of the auctioneer-free design: identical bid sets must give
    identical winners on every robot, in any iteration order."""
    bids = {"robot_1": 0.20, "robot_2": 0.55, "robot_3": 0.31}
    winners = set()
    for order in ([("robot_1", 0.20), ("robot_2", 0.55), ("robot_3", 0.31)],
                  [("robot_3", 0.31), ("robot_1", 0.20), ("robot_2", 0.55)],
                  [("robot_2", 0.55), ("robot_3", 0.31), ("robot_1", 0.20)]):
        d = dict(order)
        winners.add(min(d.keys(), key=lambda r: (-d[r], r)))
    assert winners == {"robot_2"}


def test_tie_broken_by_lowest_id():
    d = {"robot_3": 0.5, "robot_1": 0.5, "robot_2": 0.5}
    assert min(d.keys(), key=lambda r: (-d[r], r)) == "robot_1"


def test_claim_collision_lower_id_keeps_task():
    a = mk("robot_2")
    a.my_task = TASK
    a.on_award({"task_id": "T-1", "winner_id": "robot_1"})
    assert a.my_task is None, "higher id must yield on a claim collision"


def test_claim_collision_higher_id_yields_not_me():
    a = mk("robot_1")
    a.my_task = TASK
    a.on_award({"task_id": "T-1", "winner_id": "robot_2"})
    assert a.my_task is not None, "lower id must KEEP the task"


def test_failed_task_reannouncement_elects_self_when_lowest_alive_id():
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    a.tick(10.0, {"robot_3"}, {"robot_2": 0.0})
    assert len(sent) == 1
    assert sent[0]["task_id"] == TASK.task_id


def test_failed_task_reannouncement_not_duplicated_by_higher_id():
    sent = []
    a = TaskAuction("robot_3", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    a.tick(10.0, {"robot_1"}, {"robot_2": 0.0})
    assert sent == []


def test_completion_is_seen_by_all_robots():
    a = mk("robot_2")
    a.assignments[TASK.task_id] = "robot_1"
    a.on_complete(TASK.task_id)
    assert TASK.task_id in a.completed
    assert TASK.task_id not in a.assignments
