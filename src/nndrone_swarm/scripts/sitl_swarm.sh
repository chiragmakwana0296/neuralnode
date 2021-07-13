#!/bin/bash

PARAM_FILE0=$(rospack find nndrone_swarm)/param/nndrone_sitl1.param
PARAM_FILE1=$(rospack find nndrone_swarm)/param/nndrone_sitl2.param
PARAM_FILE2=$(rospack find nndrone_swarm)/param/nndrone_sitl3.param

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --wipe-eeprom --add-param-file=$PARAM_FILE0 -m --mav10 --console --mavproxy-args=\"--streamrate=50\" -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --wipe-eeprom --add-param-file=$PARAM_FILE1 -m --mav10 --console --mavproxy-args=\"--streamrate=50\" -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --wipe-eeprom --add-param-file=$PARAM_FILE2 -m --mav10 --console --mavproxy-args=\"--streamrate=50\" -I2" \
