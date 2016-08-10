#!/bin/bash

#Works on Linux alone at the moment.

OPENCV_DEPS=moret2/packages/install_opencv_deps.sh
AR_DIR=moret2/packages/ARToolKitPlus-2.2.1
MORET2_DIR=moret2


opencv_deps:
	chmod u+x $(OPENCV_DEPS)
	./$(OPENCV_DEPS)

opencv_tool: opencv_deps
	wget wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.11/opencv-2.4.11.zip
	unzip opencv-2.4.11.zip

opencv: opencv_tool
	cd opencv-2.4.11/
	mkdir build && cd build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local   -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON  -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D BUILD_SHARED_LIBS=OFF ..
	make
	sudo make install

#Install ARtoolkitplus
libartoolkitplus:
	cd $(AR_DIR) && mkdir build && cd build
	cmake ../
	make

moret2_build:
	mkdir -p $(MORET2_DIR)/build && cd $(MORET2_DIR)/build 

