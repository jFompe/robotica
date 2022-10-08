# Makefile for ROS2 docker containers
#
# Usage:
#   make <target> <arg-name>=<arg-value>
#
# Examples:
#		make build-all
#   make coppelia
#   make rviz



# BUILD
build-base:
	@docker build -f ${BASE_DOCKERFILE} -t nvidia_ros_base .

build-coppelia:
	@docker-compose build coppelia

build-rviz:
	@docker-compose build rviz

build-all: build-coppelia build-rviz



# RUN
base-run:
	@xhost +local:docker

coppelia: base-run
	@docker-compose up coppelia

rviz: base-run
	@docker-compose up rviz

all: coppelia rviz