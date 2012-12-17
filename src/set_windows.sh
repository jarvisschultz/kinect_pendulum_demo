#!/bin/bash
sleep 2

###############
# MOVIE SIZE: #
###############
# # rgb window:
# wmctrl -r /camera/rgb/image_color -e 0,1600,0,400,250
# # depth window:
# wmctrl -r /camera/depth/image -e 0,2000,0,400,250
# # move the demo window:
# wmctrl -r skeleton_interface.py -e 0,1600,275,800,300


#########################
# EXTERNAL MONITOR SIZE #
#########################
# rgb window:
wmctrl -r /camera/rgb/image_color -e 0,1600,0,800,600
# depth window:
wmctrl -r /camera/depth/image -e 0,2400,0,800,600
# move the demo window:
wmctrl -r skeleton_interface.py -e 0,1600,600,1600,600

