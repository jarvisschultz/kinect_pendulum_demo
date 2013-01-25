#!/bin/bash

win1=skeletontracker
win2=/camera/depth/image
win3=skeleton_interface.py
for i in {1..5}
do
    echo "checking for window existence... "${i}
    exist1=`wmctrl -l |grep -c $win1`
    exist2=`wmctrl -l |grep -c $win2`
    exist3=`wmctrl -l |grep -c $win3`
    if  (( $exist1 == 1 )) && (( $exist2 == 1 )) && (( $exist3 == 1 )) 
	then 
	echo "window exists!"
	break
    else
	sleep 1
    fi
done

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
# wmctrl -r /camera/rgb/image_color -e 0,1600,0,800,600
wmctrl -r $win1 -e 0,1600,0,800,600
# depth window:
wmctrl -r $win2 -e 0,2400,0,800,600
# move the demo window:
wmctrl -r $win3 -e 0,1600,600,1600,600

exit 0
