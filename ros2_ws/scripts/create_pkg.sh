#!/bin/bash


PKG_NAME=$1
[[ -z $PKG_NAME ]] && echo "ERROR: Missing package name arg" && exit 1


( \
cd ../src && \
ros2 pkg create \
  --build-type ament_python \
  --license MIT \
  $PKG_NAME && \
chmod -R 777 $PKG_NAME
)
