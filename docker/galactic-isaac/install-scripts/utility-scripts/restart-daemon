#!/usr/bin/env bash
pid=$(ps -C _ros2_daemon --no-headers | awk '{print $1}')
kill -9 $pid
ros2 daemon start