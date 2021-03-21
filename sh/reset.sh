# pkill -f main.py
rosservice call /gazebo/reset_simulation "{}"
python3 ~/move-in-maze/main.py
