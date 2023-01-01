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

build-nav:
	@docker compose build nav2

build-all: build-coppelia build-rviz build-slam



# RUN
base-run:
	@xhost +local:docker

coppelia: base-run
	@docker compose up coppelia

rviz: base-run
	@docker compose up rviz

.PHONY: nav2
nav2:
	@docker compose up nav2

slam:
	@docker compose up slam

all: coppelia rviz slam



# CONNECT TO CONTAINERS
connect-coppelia:
	@docker exec -it ros2coppelia bash -c "./ros_entrypoint.sh bash"

connect-rviz:
	@docker exec -it ros2rviz bash -c "/ros_entrypoint.sh bash"

connect-nav2:
	@docker exec -it ros2nav2 bash -c "/ros_entrypoint.sh bash"

connect-slam:
	@docker exec -it ros2slam bash -c "/ros_entrypoint.sh bash"



# UTILS
tf-graph:
	@docker exec ros2slam bash -c "cd graphs && /ros_entrypoint.sh ros2 run tf2_tools view_frames"