#!/bin/bash

rosrun xacro xacro.py `rospack find bioloid_master`/urdf/bioloid.xacro -o bioloid.urdf &&
check_urdf bioloid.urdf &&
urdf_to_graphiz bioloid.urdf
