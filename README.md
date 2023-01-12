# ROS2 with Docker

Coppelia and RViz docker images with GUI support. (Tested in Ubuntu 22.04)

## Pre-requisites

[Docker](https://docs.docker.com/engine/install/ubuntu/) and the [docker compose plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository) are needed.


## Building and running

Make is used to facilitate the process of building and running the services.

### Available services:
- coppelia
- rviz
- nav2
- slam

### Build

- A single service: `make build-{service}`. For example: `make build-coppelia`
- Build all services: `make build-all`


### Run

- Each service in a separate terminal: `make {service}`.

```bash
make coppelia
make rviz
make nav2
make slam
```

Then run the simulation in Coppelia to start sending laser data.

Finally, send the goal pose:

```bash
make start-goal
```
