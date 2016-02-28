#!/bin/bash


echo "Type in foldername"
ls ../Batch
read name

mkdir -p ../Batch/$name

cp -R ../images_* ../Batch/$name/
