# Distributed task completion announcement.
# Every robot marks this task terminal and removes any stale assignment.
std_msgs/Header header
string task_id
string robot_id
float64 stamp
