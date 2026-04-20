#!/bin/sh
pactl set-default-sink 1
pactl set-default-source 2
python3 /home/orangepi/Desktop/Server_final/main.py
read var
