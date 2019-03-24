#!/bin/zsh

source ~/.zshrc
killall -9 roscore
killall -9 rosmaster
killall gzclient
killall gzserver
roscore &
roscd morus_gui
python3 src/gui_main.py
