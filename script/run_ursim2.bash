#!/usr/bin/env bash

# Create ursim network
docker network create --subnet=192.168.2.0/24 ursim_net2;

# run ursim
docker run -it \
	--rm \
	--name ursim2 \
	--net ursim_net2 \
	--ip 192.168.2.120 \
	-e ROBOT_MODEL=UR3 \
	-v ./:/urcaps \
	universalrobots/ursim_e-series