# This message holds additional data
# the unit of acceleration is in g (linear acceleration)
# the units of torques is Nm
# the units of temperatures is degree C
# the units of current is A

string[] name
geometry_msgs/Vector3Stamped[] acceleration
time torque_time
float64[] torque
time temperature_time
float64[] temperature
time current_time
float64[] current
