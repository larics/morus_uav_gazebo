killall -9 roscore
killall -9 rosmaster
roscore &
python3 ../src/gui_main.py
