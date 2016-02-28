
import time
import sys, ast, getopt, types
import threading
# sys.path.append("../packages/live555-1.0/build/lib.macosx-10.6-intel-3.4")
# import live555
import numpy,os
# import matplotlib.pyplot as plt
import signal
import cv2

import subprocess as sp
import atexit


FFMPEG_BIN = 'ffmpeg'

command = [ FFMPEG_BIN,
            '-i', sys.argv[1],
            '-f', 'image2pipe',
            '-pix_fmt', 'rgb24',
            '-s', '640x480',
            '-r', '12', 
            '-threads', '4',
            '-vcodec', 'rawvideo', '-']
pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8,preexec_fn=os.setsid)

# stdoutdata, stderrdata = pipe.communicate()
# print(pipe.returncode)

seconds = 150

# NOTE: the username & password, and the URL path, will vary from one
# camera to another!  This URL path works with the Lorex LNB2153:
# url = 'rtsp://admin:rib919@169.254.218.160/live.sdp'

# # fOut = open(fileOut, 'wb')

# def oneFrame(codecName, bytes, sec, usec, durUSec):
#   print('frame for %s: %d bytes' % (codecName, len(bytes)))
#   if bytes:
#       pipe.stdin.write( b'\0\0\0\1' + bytes )
#       time.sleep(0.001)
#       raw_image = pipe.stdout.read(420*360*3)
#       print('check')
      # # fOut.write(b'\0\0\0\1' + bytes)
      # image =  numpy.fromstring(raw_image, dtype='uint8')
      # image = image.reshape((360,420,3))

      # plt.imshow(image, cmap='hot')
      # plt.show()
def killall():
    os.killpg(pipe.pid, signal.SIGTERM)
atexit.register(killall)
# # Starts pulling frames from the URL, with the provided callback:
# useTCP = False
# live555.startRTSP(url, oneFrame, useTCP)

# # Run Live555's event loop in a background thread:
# t = threading.Thread(target=live555.runEventLoop, args=())
# t.setDaemon(True)
# t.start()
endTime = time.time() + seconds
# plt.ion()
# plt.show()
while 1:
    # print time.time(), endTime
    raw_image = pipe.stdout.read(640*480*3)
    # transform the byte read into a numpy array
    image = numpy.fromstring(raw_image, dtype='uint8')
    image = image.reshape((480,640,3))

    print image.shape
    (h,w,k) = image.shape
    vis2 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # throw away the data in the pipe's
    cv2.imshow('image',vis2)
    cv2.waitKey(25)
    # time.sleep(1)
    print 'check'


# Tell Live555's event loop to stop:
# live555.stopEventLoop()

# Wait for the background thread to finish:
# t.join()