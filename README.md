# target_controller
Передача топиков на desktope по SSH
Desktope терминал:
export ROS_DISCOVERY_SERVER="192.168.137.2:11811"
export ROS_SUPER_CLIENT=TRUE
ros2 topic list --no-daemon --spin-time 5
Должны появиться топики
