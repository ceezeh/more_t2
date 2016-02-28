# fp = open('names.txt', 'r')
out = open('ids.txt', 'w')

alist = [line.rstrip() for line in open('names.txt', 'r')]

# out.write(alist)
for name in alist:
  out.write("'"+name+"',")

out.close()

# import time
# import sys
# import threading
# # sys.path.append("../packages/live555-1.0/build/lib.macosx-10.6-intel-3.4")
# # import live555
# import numpy,os
# # import matplotlib.pyplot as plt
# import signal
# import struct

# import subprocess as sp


# # path =  os.path.dirname(os.path.realpath(__file__))
# # prog = path + "/temp"
# prog = "source '/Users/ceezeh/Documents/PhdBase/More_T2/src/Release/moret2/moret2/../ros_catkin_ws/install_isolated/setup.bash' && '/Users/ceezeh/Documents/PhdBase/More_T2/src/Release/moret2/moret2/../catkin_ws/devel/lib/moret2/adjustCamPose' -l '/Users/ceezeh/Documents/Tracking Jobs/test1/Tempx55j_/testcam' -s 100 -a '/Users/ceezeh/Documents/PhdBase/More_T2/src/Release/moret2/moret2/../packages/ARToolKitPlus-2.2.1/sample/data/markerboard_480-499.cfg' -i rtsp://admin:rib919@169.254.218.160/live.sdp"
# pipe = sp.Popen(prog, stdout = sp.PIPE, shell=True, bufsize=10**8,preexec_fn=os.setsid)

# # stdoutdata, stderrdata = pipe.communicate()
# # print(pipe.returncode)

# seconds = 10

# # NOTE: the username & password, and the URL path, will vary from one
# # camera to another!  This URL path works with the Lorex LNB2153:
# # url = 'rtsp://admin:rib919@169.254.218.160/live.sdp'

# # # fOut = open(fileOut, 'wb')

# # def oneFrame(codecName, bytes, sec, usec, durUSec):
# #   print('frame for %s: %d bytes' % (codecName, len(bytes)))
# #   if bytes:
# #       pipe.stdin.write( b'\0\0\0\1' + bytes )
# #       time.sleep(0.001)
# #       raw_image = pipe.stdout.read(420*360*3)
# #       print('check')
#       # # fOut.write(b'\0\0\0\1' + bytes)
#       # image =  numpy.fromstring(raw_image, dtype='uint8')
#       # image = image.reshape((360,420,3))

#       # plt.imshow(image, cmap='hot')
#       # plt.show()

# # # Starts pulling frames from the URL, with the provided callback:
# # useTCP = False
# # live555.startRTSP(url, oneFrame, useTCP)

# # # Run Live555's event loop in a background thread:
# # t = threading.Thread(target=live555.runEventLoop, args=())
# # t.setDaemon(True)
# # t.start()
# endTime = time.time() + seconds
# # plt.ion()
# # plt.show()
# while time.time() < endTime:
#     # print time.time(), endTime
#     nums = pipe.stdout.read(8)
#     p,y = struct.unpack('ff',nums)
#     print "yaw: {}, pitch: {}".format(p,y)
#     # time.sleep(1)
#     print 'check'

# os.killpg(pipe.pid, signal.SIGTERM)
# # Tell Live555's event loop to stop:
# # live555.stopEventLoop()

# # Wait for the background thread to finish:
# # t.join()
# exit()