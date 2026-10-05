# R6: abort a task BEFORE pickup. Published on /fleet/task_cancel (COORD_QOS)
# by the webapp (sih-57) or any operator tool. There is NO ack message: the
# webapp infers ABORTED when the holder's RobotState.task_id clears, and
# REFUSED when the robot continues to the dropoff (already loaded).
std_msgs/Header header
string task_id        # the task to abort
string requester_id   # who asked (e.g. "webapp")
string reason         # free text, logged by the holder
