#!/bin/bash


PKG_NAME=$1
[[ -z $PKG_NAME ]] && echo "ERROR: Missing package name arg" && exit 1


PKG_DIR=src/$PKG_NAME


ros2 pkg create \
  --package-format 3 \
  --license MIT \
  --destination-directory src/ \
  --build-type ament_python \
  --maintainer-email "jaime.fompe@gmail.com" \
  --maintainer-name "Jaime Fomperosa" \
  --node-name $PKG_NAME \
  $PKG_NAME

chmod -R 777 $PKG_DIR


mkdir $PKG_DIR/launch
cp scripts/src/launch.py $PKG_DIR/launch/$PKG_NAME.launch.py

mkdir $PKG_DIR/rviz
cp scripts/src/urdf_config.rviz $PKG_DIR/rviz/urdf_config.rviz


# TODO for loop
mkdir $PKG_DIR/models
cp scripts/src/models/* $PKG_DIR/models/

mkdir $PKG_DIR/worlds
cp scripts/src/worlds/* $PKG_DIR/worlds/