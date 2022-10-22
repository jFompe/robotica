#!/bin/sh



case $1 in
  "build")
    colcon build --symlink-install
  ;;
  "source")
    . install/setup.sh
  ;;
esac