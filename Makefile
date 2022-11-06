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

build-slam:
	@docker compose build slam

build-all: build-coppelia build-rviz build-slam



# RUN
base-run:
	@xhost +local:docker

coppelia: base-run
	@docker compose up coppelia

rviz: base-run
	@docker compose up rviz

slam:
	@docker compose up slam

all: coppelia rviz slam



# CONNECT TO CONTAINERS
connect-coppelia:
	@docker exec -it ros2coppelia bash -c "./ros_entrypoint.sh bash"

connect-rviz:
	@docker exec -it ros2rviz bash -c "/ros_entrypoint.sh bash"

connect-slam:
	@docker exec -it ros2slam bash -c "/ros_entrypoint.sh bash"