#!/bin/bash

path_to_collada="../../motion/arcsys2_description/urdf/arcsys2.dae"
iktype="translationdirection5d"
output_path="src/ikfast_moveit_plugin.cpp"

# Arguments of --baselink and --eelink are index number of link
# The following command will helps you to determine the index number
#   $ openrave-robot.py <path_to>.dae --info links

python $(openrave-config --python-dir)/openravepy/_openravepy_/ikfast.py --robot=$path_to_collada --iktype=$iktype --baselink=0 --eelink=5 --savefile=$output_path
