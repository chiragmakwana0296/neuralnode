#!/bin/bash
PARAM_FILE=$1
sim_vehicle.py -v ArduCopter -f gazebo-iris --wipe-eeprom --add-param-file=$PARAM_FILE -m --mav10 --map --console --mavproxy-args="--streamrate=50" -I0