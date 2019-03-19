killall -9 roscore
killall -9 rosmaster
killall gzclient
killall gzserver
roscore &
python3 ../src/gui_main.py
