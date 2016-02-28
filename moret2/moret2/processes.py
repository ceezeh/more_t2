from enum import Enum
import Tkinter as tk
import tkFileDialog
from xml.etree import ElementTree
from xml.etree.ElementTree import Element
from xml.etree.ElementTree import SubElement
import os, sys, pipes
import subprocess
import threading
import signal
import time
import shutil
import numpy
import struct
from PIL import Image, ImageTk
#------------Project Config Classes
class ProjectStatus(Enum):
	Pending = "Pending"
	Completed = "Completed"
class OriginType(Enum):
	Camera = "Camera"
	Marker = "Marker"
class CameraType(Enum):
	IP = "IP"
	USB = "USB"
class Camera(object):
	def __init__(self, name=None, type_t= None, address=None):
		self.name = name
		self.type = type_t
		self.address = address
		self.pose =[]
		self.calibration = Calibration()
	def configToXML(self, element):
		cameraxml = SubElement( element, 'camera', \
		name=self.name, type=self.type, address=self.address)#,\
		self.calibration.configToXML(cameraxml)
			# pose = self.pose, calibration=self.calibration)
		if self.pose:
			SubElement(cameraxml, 'pose', value=','.join([str(v) for v in self.pose]))
		return cameraxml
	def readfromNode(self,node):
		self.name = node.attrib[ 'name' ]
		self.type = node.attrib[ 'type' ]
		self.address = node.attrib[ 'address' ]
		try:
			self.calibration.readfromNode(node.find('calibration'))
		except:
			print "can not read calibration"
		try: 
			pose = node.find('pose').attrib['value']
			print pose
			pose = pose.split(",")
			print pose
			self.pose = [float(p) for p in pose]
			print pose
		except:
			print "can not read pose"
		# self.pose = node.attrib[ 'pose' ]
		# self.calibration = node.attrib[ 'calibration' ]
		
class Cameras(object):
	def __init__(self):
		self.cameras ={}
	def addCamera(self, cam):
		self.cameras[cam.name] = cam
	def configToXML(self, element):
		camerasxml = SubElement( element, 'cameras')
		for key in self.cameras:
			self.cameras[key].configToXML(camerasxml)
		return camerasxml
	def readfromNode(self, node):
		# try:
			for child in node.getchildren():
				camera = Camera()
				camera.readfromNode(child)
				self.addCamera(camera)
		# except:
		# 	print "error in reading cameras"
class Calibration(object):
	def __init__(self): 
		self.tags = ['PrincipalX','PrincipalY','focus_lenX','focus_lenY', 'Dist1','Dist2','Dist3','Dist4']
		self.alltags = ['x','y','PrincipalX','PrincipalY','focus_lenX','focus_lenY', 'Dist1','Dist2','Dist3','Dist4']
		self.calib ={}
	def configToXML(self, element):
		calibxml = SubElement(element, 'calibration')
		try:
			for key, value in self.calib.iteritems():
				print key, value
				SubElement(calibxml, key, value=str(value))
		except:
			print "Can not store calibration"
		return calibxml

	def readfromFile(self, filepath):
		document = ElementTree.parse( filepath )
		calibDOM = document.getroot()
		resultsDOM = document.find( 'results')
		for tag in self.tags:
			if tag in ['Dist1','Dist2','Dist3','Dist4']:
				self.calib[tag] = float("{:.6f}".format(float(resultsDOM.find(tag).text)))
			else:
				self.calib[tag] = float("{:.3f}".format(float(resultsDOM.find(tag).text)))
	def readfromNode(self, node):
		for child in node.getchildren():
			print "Calib bode name %s" %child.tag
			if child.tag == "x" or child.tag == "y":
				self.calib[child.tag] = int(child.attrib['value'])
			else:
				self.calib[child.tag] = float(child.attrib['value'])
	def writeToARFile(self, filename):
		target = open(filename,'w+')
		target.write('ARToolKitPlus_CamCal_Rev02\n')
		cal = " ".join([str(self.calib[tag]) for tag in self.alltags])
		cal += " 0.0 0.0 0 20"
		target.write(cal)
		target.close()
class ProjectConfig(object):
	def __init__ (self):
		self.status = ProjectStatus.Pending
		self.cameras = Cameras()
		self.markersize =int()
		self.dirname = str()
		self.jobs = [] # A list of capture jobs referenced by name
	def configToXML(self):
		configxml= Element( 'config' )
		self.cameras.configToXML(configxml)
		SubElement(configxml, 'status', value= self.status.value)
		SubElement(configxml, 'markersize', value= str(self.markersize))
		SubElement(configxml, 'dirname', value= str(self.dirname))
		jobsxml = SubElement(configxml, 'jobs')
		for job in self.jobs:
			SubElement(jobsxml, 'job', value= job)
		return  configxml
	def readFile(self, filepath):
		document = ElementTree.parse( filepath )
		configDOM = document.getroot()
		camerasDOM = document.find( 'cameras')
		self.cameras.readfromNode(camerasDOM)

		self.markersize = int(document.find( 'markersize' ).attrib['value'])
		self.status = ProjectStatus[document.find( 'status' ).attrib['value']]
		self.dirname = document.find( 'dirname' ).attrib['value']

		for node in document.findall( 'jobs/job' ):
			self.jobs.append(node.attrib['value'])
#-------------Load project---------------
# def loadProject(config):
# 	file_opt = options = {}
# 	options['title'] = 'Please select a project xml file'
# 	options['initialdir'] = config.dirname
# 	options['filetypes'] = [('xml files', '.xml')]
# 	filename = tkFileDialog.askopenfilename(**file_opt)
# 	if len( filename ) > 0:
# 		print "You chose %s" % filename 
# 	# Read xml file.

# def saveProject(config):
# 	configxml = config.configToXML()
# 	dir_opt = options = {}
# 	options['title'] = 'Select Project Name'
# 	options['initialdir'] = config.dirname

# 	projectname = tkFileDialog.asksaveasfilename(**dir_opt)
# 	print "Dirname %s" % projectname 
# 	if os.path.exists(projectname ):
# 	    pass
# 	else:
# 	    os.mkdir( projectname , 0755 )
# 	xmlfilename = projectname + "/config.xml"
# 	output_file = open( xmlfilename, 'w' )
# 	output_file.write( '<?xml version="1.0"?>' )
# 	output_file.write( ElementTree.tostring( configxml ) )
# 	output_file.close() 


class RecordWorker(object):
	def __init__(self,jobname, selectedCams, config):
		self.jobname = jobname
		self.selectedCams = selectedCams
		self.config = config
		self.procList = []
		self.worker = threading.Thread(target=self.recordVideo) 
		self.worker.start()
	def endall(self):
		exit_codes = [p.terminate() for p in self.procList]

	def recordVideo(self):
		jobpath = self.config.dirname + "/" + self.jobname
		if os.path.exists(jobpath ):
			pass
		else:
			os.mkdir( jobpath , 0755 )
		
		for cam in self.selectedCams:
			# Get Address
			address = self.config.cameras.cameras[cam].address
			vidpath = jobpath + "/" + cam

			if os.path.exists(vidpath ):
				pass
			else:
				os.mkdir( vidpath , 0755 )

			vidpath = pipes.quote(vidpath)
			vidpath ="{}".format(vidpath)
			

			livepath = os.path.dirname(os.path.realpath(__file__)) + "/../packages/live/testProgs/openRTSP"
			livepath = pipes.quote(livepath)

			# Create .cal file out of calibration data.
			cmd = "{} -v -i -L ".format(livepath) +vidpath + " "+ address
			print cmd
			proc = subprocess.Popen(cmd, universal_newlines=True, shell=True, executable="/bin/bash", stdin=subprocess.PIPE)
			self.procList.append(proc)
		exit_codes = [p.wait() for p in self.procList]

class ProcessWorker(object):
	def __init__(self, jobnames, selectedCams, config, markerList=None,unknownCam=None):
		
		self.unknownCam = unknownCam
		self.selectedCams = selectedCams
		self.config = config
		self.jobnames = jobnames
		self.procList = []
		self.markerList = markerList
		self.endFlag = False
		self.worker = threading.Thread(target=self.processVideo)
		self.worker.start()
	def endall(self):
		# os.killpg(pro.pid, signal.SIGTERM)

		exit_codes = [os.killpg(p.pid, signal.SIGTERM) for p in self.procList]
		self.endFlag = True
		self.procList =[]
	def processVideo(self):
		for jobname in self.jobnames:
			jobpath = self.config.dirname + "/" + jobname
			for cam in self.selectedCams:
				vidpath = jobpath + "/" + cam +"/vid.avi"
				
				vidpath = pipes.quote(vidpath)
				vidpath ="{}".format(vidpath)
				outpath = jobpath + "/" + cam +"/vid.mov"
				outpath = pipes.quote(outpath)
				outpath ="{}".format(outpath)
				cmd =  'ffmpeg  -i '+ vidpath+'  '+outpath+' -y && rm  '+vidpath
				print cmd
				proc = subprocess.Popen(cmd, universal_newlines=True, shell=True, executable="/bin/bash", stdin=subprocess.PIPE,preexec_fn=os.setsid)
				self.procList.append(proc)
			exit_codes = [p.wait() for p in self.procList]
			if self.endFlag:
				break

			#New!! Undistort using matlab function
			for cam in self.selectedCams:
				cmd = pipes.quote(os.path.dirname(os.path.realpath(__file__)) + "/../source/undistort.sh")
				vidpath1=jobpath + "/" + cam + "/vid_old.mov"
				vidpath2=jobpath + "/" + cam + "/vid.mov"
				if os.path.exists(vidpath1 ) and os.path.exists(vidpath2 ):
					pass
				else:
					arg = jobpath + "/" + cam 
					arg = pipes.quote(arg)
					cmd += " " +arg
					print cmd
					proc = subprocess.Popen(cmd, universal_newlines=True, shell=True, executable="/bin/bash", stdin=subprocess.PIPE,preexec_fn=os.setsid)
					self.procList.append(proc)
				exit_codes = [p.wait() for p in self.procList]
			if self.endFlag:
				break
			#Create *.cal file.


			genposepath = os.path.dirname(os.path.realpath(__file__)) + "/../bin/genpose"
			genposepath = pipes.quote(genposepath)
			# rossourcepath = os.path.dirname(os.path.realpath(__file__)) +"/../ros_catkin_ws/install_isolated/setup.bash"
			# rossourcepath = pipes.quote(rossourcepath)
			arconfigpath = os.path.dirname(os.path.realpath(__file__)) +"/../packages/ARToolKitPlus-2.2.1/sample/data/markerboard_480-499.cfg"
			arconfigpath = pipes.quote(arconfigpath)
			markerID_str = ''

			self.procList =[]
			#store camera calibration for removal
			calibrationFiles = []
			for val in self.markerList:
				markerID_str += " " + str(val)
			for cam in self.selectedCams: #cam is camera name
				camjobpath = jobpath + "/" + cam
				calpath = camjobpath +"/ar_calibration.cal"
				# calpath = pipes.quote(calpath)
				self.config.cameras.cameras[cam].calibration.writeToARFile(calpath)
				calibrationFiles.append(calpath)
				camjobpath = pipes.quote(camjobpath)

				originFlag = 0
				resultFlag = 0
				# pass in calibration data:
				# cmd = "source " +rossourcepath
				cmd = genposepath+" -c "+ cam + " -l "+camjobpath + " -s " + str(self.config.markersize) + " -a " + arconfigpath
				if markerID_str:
					cmd+= " -m \'"+ markerID_str + "\'"
				# Add marker pose here.
				if cam !=self.unknownCam:
					#Add camera pose here ...
					pose_str = " ".join([str(val) for val in self.config.cameras.cameras[cam].pose])
					pose_str = pipes.quote(pose_str)
					cmd += " -p " + pose_str
					print cmd
				proc = subprocess.Popen(cmd,  universal_newlines=True, shell=True, executable="/bin/bash",preexec_fn=os.setsid)
				self.procList.append(proc)
				
				
			exit_codes = [p.wait() for p in self.procList]
			exit_cmds = [ os.remove(path) for path in calibrationFiles]
			if self.endFlag:
				break

class CameraPoseWorker(object):
	def __init__(self, config, knownCam=None, unknownCam=None,markerID=None,jobname=None):
		if jobname:
			self.jobname =jobname
		else:
			self.jobname = "Tempx55j_"
		print("campose worker jobname: \"", self.jobname,"\"")
		self.config = config
		self.knownCam = knownCam
		self.unknownCam = unknownCam
		self.markerID = markerID
		self.worker = threading.Thread(target=self.estimatePose)
		self.worker.start()
		
	def estimatePose(self):
		if self.knownCam ==None:
			camlist = [self.unknownCam]
		else:
			camlist = [self.unknownCam,self.knownCam]
		if self.jobname == "Tempx55j_":
			self.recordWorker = RecordWorker(self.jobname, camlist, self.config)
			time.sleep(3)
			print "Check!"
			self.recordWorker.endall()

		if self.markerID == None:
			markerlist = []
		else:
			markerlist = [self.markerID]

		jobs = []
		jobs.append(self.jobname)
		self.processWorker = ProcessWorker(jobs, camlist, self.config, markerlist,self.unknownCam)
		self.processWorker.worker.join()

		#Genpose starts here.
		gencamposepath = os.path.dirname(os.path.realpath(__file__)) + "/../bin/gencampose"
		gencamposepath = pipes.quote(gencamposepath)

		jobpath = self.config.dirname + "/" + self.jobname
		jobpath = pipes.quote(jobpath)

		# rossourcepath = os.path.dirname(os.path.realpath(__file__)) +"/../ros_catkin_ws/install_isolated/setup.bash"
		# rossourcepath = pipes.quote(rossourcepath)
		# cmd = "source " +rossourcepath
		
		cmd  = gencamposepath + " -l " +  jobpath + " -t "+ self.unknownCam
		if self.knownCam != None:
			cmd += " -f " + self.knownCam
		print cmd
		p = subprocess.Popen(cmd,shell=True, 
				executable="/bin/bash")
		p.wait()

		#read results:
		pose = []
		resultpath = self.config.dirname + "/" + self.jobname + "/pose.csv"
		f = open(resultpath, 'r+')
		for line in f.readlines():
			pose.append(float(line))

		self.config.cameras.cameras[self.unknownCam].pose = pose
		self.estimatedpose = pose
		jobpath = self.config.dirname + "/" + self.jobname
		# shutil.rmtree(jobpath)

class StreamerWorker(object):
	def __init__(self, config,selectedCam):
		self.config = config
		self.jobname = "Tempx55j_"
		self.selectedCam =selectedCam
		self.worker = threading.Thread(target=self.streamVideo)
		self.worker.start()
		self.runFlag = 1
	def endall(self):
		os.killpg(self.proc.pid, signal.SIGTERM)
	def poll(self):
		if hasattr(self,"proc"):
			return self.proc.poll()
		else:
			return -1
	def streamVideo(self):
		markerDetectpath = os.path.dirname(os.path.realpath(__file__)) + "/../bin/markerDetect"
		markerDetectpath = pipes.quote(markerDetectpath)
		# rossourcepath = os.path.dirname(os.path.realpath(__file__)) +"/../ros_catkin_ws/install_isolated/setup.bash"
		# rossourcepath = pipes.quote(rossourcepath)
		arconfigpath = os.path.dirname(os.path.realpath(__file__)) +"/../packages/ARToolKitPlus-2.2.1/sample/data/markerboard_480-499.cfg"
		arconfigpath = pipes.quote(arconfigpath)
		markerID_str = ''

		self.procList =[]
		#store camera calibration for removal
		calibrationFiles = []
		jobpath = self.config.dirname + "/" + self.jobname
		camjobpath = jobpath + "/" + self.selectedCam
		print "camjobpath : %s" %camjobpath
		if os.path.exists(camjobpath ):
			pass
		else:
			os.makedirs( camjobpath , 0755 )
		calpath = camjobpath +"/ar_calibration.cal"
		camjobpath = pipes.quote(camjobpath)

		self.config.cameras.cameras[self.selectedCam].calibration.writeToARFile(calpath)
		calibrationFiles.append(calpath)
		originFlag = 0
		resultFlag = 0
		# pass in calibration data:
		address =self.config.cameras.cameras[self.selectedCam].address
		# cmd = "source " +rossourcepath
		cmd = markerDetectpath+ " -l "+camjobpath + " -s " + str(self.config.markersize) + " -a " + arconfigpath + " -i "+ address
		print "Command : %s" %cmd
		self.proc = subprocess.Popen(cmd,  universal_newlines=True, shell=True, executable="/bin/bash",preexec_fn=os.setsid)
		
		exit_codes = self.proc.wait()
		shutil.rmtree(jobpath)
		
		
class PoseReader(object):
	def __init__(self,config,selectedCam):
		self.config = config
		self.jobname = "Tempx55j_"
		self.selectedCam =selectedCam
		self.worker = threading.Thread(target=self.run)
		self.worker.start()
		self.runFlag = 1
	def endall(self):
		os.killpg(self.proc.pid, signal.SIGTERM)
	def poll(self):
		if hasattr(self,"proc"):
			return self.proc.poll()
		else:
			return 1
	def read(self):
		nums = self.pipein.read(8)
		p,y = struct.unpack('ff',nums)
		print "yaw: {}, pitch: {}".format(p,y)
		return p,y
	def run(self):
		adjcamposepath= os.path.dirname(os.path.realpath(__file__)) + "/../bin/adjustCamPose"
		adjcamposepath = pipes.quote(adjcamposepath)
		# rossourcepath = os.path.dirname(os.path.realpath(__file__)) +"/../ros_catkin_ws/install_isolated/setup.bash"
		# rossourcepath = pipes.quote(rossourcepath)
		arconfigpath = os.path.dirname(os.path.realpath(__file__)) +"/../packages/ARToolKitPlus-2.2.1/sample/data/markerboard_480-499.cfg"
		arconfigpath = pipes.quote(arconfigpath)
		markerID_str = ''

		self.procList =[]
		#store camera calibration for removal
		calibrationFiles = []
		jobpath = self.config.dirname + "/" + self.jobname
		camjobpath = jobpath + "/" + self.selectedCam
		print "camjobpath : %s" %camjobpath
		if os.path.exists(camjobpath ):
			pass
		else:
			os.makedirs( camjobpath , 0755 )
		calpath = camjobpath +"/ar_calibration.cal"
		camjobpath = pipes.quote(camjobpath)

		self.config.cameras.cameras[self.selectedCam].calibration.writeToARFile(calpath)
		calibrationFiles.append(calpath)
		originFlag = 0
		resultFlag = 0
		# pass in calibration data:
		address =self.config.cameras.cameras[self.selectedCam].address
		# cmd = "source " +rossourcepath
		cmd = adjcamposepath+ " -l "+camjobpath + " -s " + str(self.config.markersize) + " -a " + arconfigpath + " -i "+ address
		print "Command : %s" %cmd
		self.proc = subprocess.Popen(cmd, shell=True,preexec_fn=os.setsid)
		self.pipein = open("/tmp/adjustcampose",'r')
		exit_codes = self.proc.wait()
		shutil.rmtree(jobpath)