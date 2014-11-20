#!/bin/bash


# ffmpeg -framerate 25 -video_size 320x240 -f v4l2  -i /dev/video0 \
# -framerate 25 -video_size 320x240 -f v4l2 -i /dev/video1 \
# -map 0:v -vf "setpts=PTS-STARTPTS" -vcodec copy -an -y ../video/video_cam_0.mkv -map 1:v -vf "setpts=PTS-STARTPTS" -vcodec copy -an -y ../video/video_cam_1.mkv
# # "[0:v]setpts=PTS-STARTPTS, pad=iw*2:ih[bg]; \
# #  [1:v]setpts=PTS-STARTPTS[fg],[bg][fg]overlay=w" output.avi

# ffmpeg -i "video_cam_0.mkv" -c copy -map 0:v "video_cam_0.mov"
# ffmpeg -i "video_cam_1.mkv"  -c copy -map 0:v "video_cam_1.mov"

# ffmpeg -f avfoundation  -pixel_format bgr0  -i "0" -f avfoundation  -pixel_format bgr0  -i "1" -s 320x240 -map 0:v ~/Documents/PhdBase/Main/Helper\ Projects/More-T2/video/video_cam_0.mov -s 320x240 -map 1:v ~/Documents/PhdBase/Main/Helper\ Projects/More-T2/video/video_cam_1.mov
runmat=0
while :
do
	if [ $runmat = 1 ]; then
	# run matlab
	echo "runing matlab!..." && sleep 2
	matlab -nosplash -r "plotpose"

	else        
    clear&&clear
    cat<<EOF
    ==============================
    More-T2 Project
    ------------------------------
    Please enter your choice:

    Get Cameras'Pose 		(1)
    Record Video 		(2)
    Generate Pose 		(3)
    (Q)uit
    ------------------------------
EOF

fi
    read -n1 -s
    case "$REPLY" in
    "1")  echo "Cameras will now be calibrated. Please wait..." && sleep 2 && rosrun more_t2 camera_pose_calibration;; 
    "2")  echo "Logging into Mac for recording Please wait ... " && sleep 2 && ssh ceezeh@chinemelus-mbp.default;;
    "3")  echo "Generating pose from recorded file. Please wait..."&& sleep 2 && runmat=1&&rosrun more_t2 more_t2;;
    "Q")  exit                      ;;
    "q")  echo "case sensitive!!"   ;; 
     * )  echo "invalid option"     ;;
    esac
    sleep 1
done