# A current assignee could not complete its route and is returning this task
# to the decentralized auction. Every robot clears the old assignment and
# starts a fresh bid window from the same task record.
std_msgs/Header header
string task_id
int32 pickup_row
int32 pickup_col
int32 dropoff_row
int32 dropoff_col
uint8 priority
float64 created_at
float64 deadline
string robot_id
string reason
