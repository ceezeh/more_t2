#!/bin/bash

path=$1
# echo 'path: '$path &&

vidpath="$path"/vid.mov
vidnewpath="$path"/vid_old.mov
mv -i "$vidpath" "$vidnewpath"
#split vid into images

rate=$(ffmpeg -i "$vidnewpath" 2>&1 | sed -n "s/.*, \(.*\) fp.*/\1/p") &&
echo 'rate: '$rate &&
imagepath="$path"/images &&
mkdir -p "$imagepath" &&
ffmpeg -r "$rate" -i "$vidnewpath" -qscale:v 2 "$imagepath"/image%010d.bmp &&

#call matlab undistort function

"/Users/ceezeh/Documents/MATLAB/undistortimages/for_testing/run_undistortimages.sh" \
/Applications/MATLAB_R2015b.app "$imagepath" \
"./../packages/undistortimages/c1_paramstruct.mat" $rate


#convert avi to mov
ffmpeg -i "$imagepath/"vid.avi "$vidpath" 

mkdir -p empty_dir/
rsync -a --delete empty_dir/ "$imagepath"
rm -rf "$imagepath"
rm -rf empty_dir