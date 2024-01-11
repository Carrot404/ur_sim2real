#!/usr/bin/env bash

# Create ursim network
docker network create --subnet=192.168.56.0/24 ursim_net;

# run ursim
docker run -it \
	--rm \
	--name ursim \
	--net ursim_net \
	--ip 192.168.56.101 \
	-e ROBOT_MODEL=UR3 \
	-v ./:/urcaps \
	universalrobots/ursim_e-series