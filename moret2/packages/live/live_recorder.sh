#!/bin/bash

function  finish {
	name=../images_*
	echo "converting video files to mov format. Please wait"
	for dir in $name
	do
	ffmpeg  -i $dir/vid.avi -q:v 1  $dir/vid.mov &
	done
}

trap finish EXIT

rm -rf ../images_*

mkdir ../images_0 ../images_1 ../images_2 ../images_3 

make

./parallel_cmds.sh "./testProgs/openRTSP -v -i -x 0 -L ../images_0/vid.avi rtsp://admin:adminadmin@192.168.10.31:554/live.sdp"  \
"./testProgs/openRTSP -v -i -x 1 -L ../images_1/vid.avi rtsp://admin:adminadmin@192.168.10.32:554/live.sdp" \
"./testProgs/openRTSP -v -i -x 2 -L ../images_2/vid.avi rtsp://admin:adminadmin@192.168.10.33:554/live.sdp" \
"./testProgs/openRTSP -v -i -x 3 -L ../images_3/vid.avi rtsp://admin:adminadmin@192.168.10.34:554/live.sdp"


# runmat=0
# while :
# do
# 	if [ $runmat = 1 ]; then
# 	# run matlab
# 	echo "runing matlab!..." && sleep 2
# 	matlab -nosplash -r "plotpose"

# 	else        
#     clear&&clear
#     cat<<EOF
#     ==============================
#     More-T2 Project
#     ------------------------------
#     Please enter your choice:

#     Start Recording 	(1)
#     Save videos  		(2)
#     Load videos 		(3)
#     (Q)uit
#     ------------------------------
# EOF

# fi
#     read -n1 -s
#     case "$REPLY" in
#     "1")  echo "Type in the id of cameras for recording." 
     		
#      		read input
#      		for id in $input
#      		do
#      		rm -f ../cfg/*
#      		./parallel_cmds.sh "rosrun more_t2 camTest1" 	"rosrun more_t2 camTest0";;
#     "2")  echo "Logging into Mac for recording Please wait ... "
#      		sleep 2 
#      		ssh -X -t ceezeh@10.111.79.249 \
#      		'cd /Users/ceezeh/Documents/PhdBase/More_T2/src/live && ./live_recorder.sh';;
#     "3")  echo "Generating pose from recorded file. Please wait..."
#     		sleep 2 
#     		runmat=1
#     		rm -f ../posedata/pose*
#     		rosrun more_t2 more_t2;;
#     "Q")  exit                      ;;
#     "q")  echo "case sensitive!!"   ;; 
#      * )  echo "invalid option"     ;;
#     esac
#     sleep 1
# done