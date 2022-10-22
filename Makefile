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
build-coppelia:
	@docker compose build coppelia

build-rviz:
	@docker compose build rviz

build-all: build-coppelia build-rviz



# RUN
base-run:
	@xhost +local:docker

coppelia: base-run
	@docker compose up coppelia

rviz: base-run
	@docker compose up rviz

ros2:
	@docker compose run ros2

all: coppelia rviz



# CONNECT TO CONTAINERS
connect-coppelia:
	@docker exec -it ros2coppelia bash -c "./ros_entrypoint.sh bash"

connect-rviz:
	@docker exec -it ros2rviz bash -c "./ros_entrypoint.sh bash"