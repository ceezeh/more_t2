#To install OpenCV 2.4.2 or 2.4.3 on the Ubuntu 12.04 operating system, first install a developer environment to build OpenCV.

    sudo apt-get -y install build-essential cmake pkg-config &&
#Install Image I/O libraries

    sudo apt-get -y install libjpeg62-dev &&
    sudo apt-get -y install libtiff4-dev libjasper-dev &&
#Install the GTK dev library

    sudo apt-get -y install  libgtk2.0-dev &&
#Install Video I/O libraries

    sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev &&
#Optional - install support for Firewire video cameras

 sudo apt-get -y install libdc1394-22-dev &&

#Optional - install video streaming libraries

 sudo apt-get -y install libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev &&
#Optional - install the Python development environment and the Python Numerical library

    sudo apt-get -y install python-dev python-numpy &&
 
 sudo apt-get install freeglut3-dev -y
#Optional - install the parallel code processing library (the Intel tbb library)

    sudo apt-get -y install libtbb-dev &&
#Optional - install the Qt dev library

    sudo apt-get -y install libqt4-dev
