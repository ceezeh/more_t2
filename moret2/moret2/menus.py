
import Tkinter as tk
import tkMessageBox
import ttk
import tkFileDialog,tkSimpleDialog
from PIL import Image, ImageTk
from processes import CameraType
from processes import OriginType
from processes import Camera
from processes import ProjectStatus


from processes import RecordWorker
from processes import ProcessWorker
from processes import CameraPoseWorker
from processes import StreamerWorker
from processes import PoseReader
from processes import ProjectConfig
import time
from copy import deepcopy

import os

from xml.etree import ElementTree
from xml.etree.ElementTree import Element
from xml.etree.ElementTree import SubElement

import subprocess
import signal
import threading

# class ImageFrame(tk.Frame):
# 	def __init__(self, parent, imagePath):
# 		tk.Frame.__init__(self, parent)
# 		self.columnconfigure(0,weight=1)
# 		self.rowconfigure(0,weight=1)
# 		self.original = Image.open(imagePath)
# 		self.image = ImageTk.PhotoImage(self.original)
# 		self.display = tk.Canvas(self, bd=0, highlightthickness=0)
# 		self.display.create_image(0, 0, image=self.image, anchor=tk.NW, tags="IMG")
# 		self.display.grid(row=0, sticky=tk.W+tk.E+tk.N+tk.S)
# 		self.pack(fill=tk.BOTH, expand=1)
# 		self.bind("<Configure>", self.resize)

# 	def resize(self, event):
# 		size = (event.width, event.height)
# 		resized = self.original.resize(size,Image.ANTIALIAS)
# 		self.image = ImageTk.PhotoImage(resized)
# 		self.display.delete("IMG")
# 		self.display.create_image(0, 0, image=self.image, anchor=tk.NW, tags="IMG")
		
class ResizingCanvas(tk.Canvas):
	def __init__(self,parent,**kwargs):
		tk.Canvas.__init__(self,parent,**kwargs)
		self.bind("<Configure>", self.on_resize)
		self.height = self.winfo_reqheight()
		self.width = self.winfo_reqwidth()

	def on_resize(self,event):
		# determine the ratio of old width/height to new width/height
		if event.width > self.width:
			wscale = float(event.width)/self.width
			hscale = float(event.height)/self.height
			self.width = event.width
			self.height = event.height
			# resize the canvas 
			self.config(width=self.width, height=self.height)
			# rescale all the objects tagged with the "all" tag
			self.scale(self,0,0,wscale,hscale)

class ViewerFrame(tk.Frame):
	def __init__(self,parent,config):
		tk.Frame.__init__(self,parent)
		self.config = config
		# frame = tk.Frame(self)
		# frame.grid(row=0,column=0,columnspan=3, sticky=tk.NSEW)
		# self.displayCanvas = ResizingCanvas(frame,width=600, height=450, bg = 'black')
		# self.configure(background='grey')
		# self.displayCanvas.pack(fill=tk.BOTH, expand=tk.YES)

		self.buttonFrame = tk.Frame(self)
		self.buttonFrame.grid(row=1, column=1)
		self.toggleViewButton = ButtonFactory(self.buttonFrame, "Switch off", self.switchoff)
		self.toggleViewButton.grid(row=0,column=0,sticky=tk.W)
		self.streamOnFlag = 0


		self.cameraMenuButton = tk.Menubutton(self.buttonFrame, text='Select Cameras',
							 relief=tk.RAISED, width=15, justify=tk.CENTER)

		self.cameraMenu =tk.Menu(self.cameraMenuButton, tearoff=0,postcommand=self.updateCameraMenu)
		self.cameraMenuButton['menu'] = self.cameraMenu

		# last = self.cameraMenu.index("end")
		# for i in range(last+1):
		# 		self.cameraMenu.entryconfig(i,state='disabled')

		self.cameraMenuButton.grid(row=0,column=1, sticky=tk.W)
		self.columnconfigure(0, weight=1)
		self.columnconfigure(2, weight=1)
		self.rowconfigure(0, weight=1)
		self.rowconfigure(1, weight=0)

	def updateCameraMenu(self):
		self.cameraMenu.delete(0,tk.END)

		for cam in self.config.cameras.cameras:
			self.cameraMenu.add_radiobutton(label=cam, command=lambda val=cam: self.viewCamera(val))
		print 'Check'
	def viewCamera(self,val):
		if hasattr(self, 'cameraName'):
			if self.cameraName != val or (self.streamerWorker.poll() != None):
				try:
					self.endall
				except:
					print "No streaming to end"
				self.cameraName = val
				key = self.cameraName
				self.streamerWorker = StreamerWorker(self.config,key)
		else:
			self.cameraName = val
			key = self.cameraName
			self.streamerWorker = StreamerWorker(self.config,key)

	def switchoff(self):
		try:
			self.streamerWorker.endall()
		except:
			print 'nothing to end'

class ButtonFactory(tk.Button):
	def __init__(self, parent, txt, fn):
		tk.Button.__init__(self,parent, justify=tk.LEFT,anchor=tk.W,  width=15,height=1,text=txt, command=fn)

class ButtonFactory2(tk.Button):	
	def __init__(self, parent,txt,fn, **kwargs):
		tk.Button.__init__(self,parent, anchor=tk.W,  width=15,height=1,text=txt, command=fn, **kwargs)

class CameraDropDownMenu(tk.Menubutton):
	def __init__(self, parent, config= None):
		tk.Menubutton.__init__(self, parent,text='Select Cameras',
							 relief=tk.RAISED, width=15, justify=tk.CENTER)

		self.menu = tk.Menu(self, tearoff=0)
		self['menu'] = self.menu
		# ToDo: Read from config file. Hard code for now.
		if config is not None:
			self.cameraList = deepcopy(config.cameras.cameras)
		else:
			self.cameraList = {}
		# for cam in cameraList:
		# 	check = tk.StringVar()
		# 	self.menu.add_command(label=cam, command=lambda val=cam: cmd)


class CameraMenuInsertToList(CameraDropDownMenu):
	def __init__(self, parent, listbox, config=None):
		CameraDropDownMenu.__init__(self,parent,config)
		self.listbox = listbox
		for cam in self.cameraList:
			self.menu.add_command(label=cam, command=lambda val=cam: self.addCameraToList(val))
			self.addCameraToList(cam)
	def addCameraToList(self, val):
		if (val not in  self.listbox.get(0, tk.END)):
			 self.listbox.insert(tk.END, val)
class CameraListFrame(tk.Frame):
	def __init__(self, parent, fn=None):
		tk.Frame.__init__(self, parent)

		self.listbox = tk.Listbox(self,selectmode=tk.EXTENDED)
		self.listbox.grid(row=1,column=0,rowspan=4)

		self.delbutton = ButtonFactory2(self,'Delete',lambda lb=self.listbox: self.deleteFromCameraList(lb, fn), justify=tk.CENTER)
		self.delbutton.grid (row=6,column=0)

	def deleteFromCameraList(self,camlist, fn=None):
		if fn is not None:
			fn()
		items = camlist.curselection()

		for item in items:
			camlist.delete(int(item))
		
class JobListFrame(tk.Frame):
	def __init__(self, parent, config, fn=None):
		tk.Frame.__init__(self, parent)
		self.config = config

		tk.Label(self, text="Job List").grid(row=0)

		self.listbox = tk.Listbox(self,selectmode=tk.EXTENDED)
		self.listbox.grid(row=1,column=0,rowspan=4)
		#initially populate the list.
		for job in self.config.jobs:
			self.listbox.insert(tk.END, job)
		self.delbutton = ButtonFactory2(self,'Delete',lambda lb=self.listbox: self.deleteFromJobList(lb, fn), justify=tk.CENTER)
		self.delbutton.grid (row=5)


		self.jobmenubutton = tk.Menubutton(self,text='Add Jobs',
							 relief=tk.RAISED, width=20, justify=tk.CENTER)

		self.jobmenubutton.menu = tk.Menu(self.jobmenubutton, tearoff=0)
		self.jobmenubutton['menu'] = self.jobmenubutton.menu
		for job in self.config.jobs:
			self.jobmenubutton.menu.add_command(label=job, command=lambda val=job: self.addToJobList(val))

		self.jobmenubutton.grid(row=6)
	def addToJobList(self, val):
		if (val not in  self.listbox.get(0, tk.END)):
			 self.listbox.insert(tk.END, val)
	def deleteFromJobList(self,joblist, fn=None):
		if fn is not None:
			fn()
		items = joblist.curselection()

		for item in items:
			joblist.delete(int(item))

def trackerWidget(config):
		parent=None
		tp = tk.Toplevel()
		tp.resizable(width=tk.FALSE, height=tk.FALSE)
		tp.title("Track")
		tp.trackerWidget = TrackerWidgetFrame(tp,config)
		# tp.protocol("WM_DELETE_WINDOW", lambda: tp.trackerWidget.closingsaverequest())
		
		tp.trackerWidget.pack()


class TrackerWidgetFrame(tk.Frame):
	def __init__(self,parent,config):
		tk.Frame.__init__(self, parent)
		self.parent = parent
		self.config = config

		frame1 = tk.Frame(self)
		frame1.grid(row=0)
		tk.Label(frame1, text="Job name:").grid(row=0, column=0, sticky=tk.W)
		self.jobLabel = tk.Label(frame1, text="N/A")
		self.jobLabel.grid(row=0, column=2, sticky=tk.W)

		tk.Label(frame1, text="Marker IDs:").grid(row=1, column=0, sticky=tk.W)
		
		# markerIDs = tk.StringVar()
		self.markerIDEntry = tk.Entry(frame1, width=20, textvariable=tk.StringVar())
		self.markerIDEntry.grid(row=1, column=2, sticky=tk.W)
		
		frame2 = tk.Frame(self)
		frame2.grid(row=1)

		frameCamList = tk.Frame(frame2)
		frameCamList.grid(row=0,column=0)
		tk.Label(frameCamList , text="Camera List").grid(row=0,column=0)
		self.cameraListFrame = CameraListFrame(frameCamList)
		self.cameraListFrame.grid(row=1,column=0, rowspan=4)
		self.listbox = self.cameraListFrame.listbox
		
		self.camMenu = CameraMenuInsertToList(frameCamList ,self.listbox, config)
		self.camMenu.grid(row=5, column=0)
		ttk.Separator(frame2 , orient=tk.VERTICAL).grid(row=2, column=1, rowspan=6, sticky=tk.NS, padx=5)
		#Load job
		self.joblist = JobListFrame(frame2, self.config)
		self.joblist.grid(row=0, column=2)


		ttk.Separator(self , orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=3, sticky=tk.EW, pady=10)

		frame3 = tk.Frame(self)
		frame3.grid(row=4)
		# Capture Button
		self.captureButton = ButtonFactory(frame3,'Capture', self.capturerequest)
		self.captureButton.grid(row=3,column=0, pady=10)
		#Process
		self.processButton = ButtonFactory(frame3,'Process', self.processrequest)
		self.processButton.grid(row=3,column=2, pady=10)
		
	def processrequest(self):
		markerlist = self.markerIDEntry.get().split(",")
		self.processWorker = ProcessWorker(self.joblist.listbox.get(0,tk.END),self.listbox.get(0,tk.END),self.config, markerlist)
		self.processButton.config(text="Stop", command=self.stopprocessrequest)
	def stopprocessrequest(self):
		try:
			self.processWorker.endall()
		except Exception,e :
			print str(e)
		self.processButton.config(text="Process", command=self.processrequest)
	def capturerequest(self):
		name = tkSimpleDialog.askstring("Capture Name?", "Please type in the capture name")
		self.pack()
		# self.lift()
		# self.call('wm', 'attributes', '.', '-topmost', True)
		# self.after_idle(self.win.call, 'wm', 'attributes', '.', '-topmost', False)

		if (len(name)>0):
			self.jobLabel.config(text=name)
			# Add job to config
			if name not in self.config.jobs:
				self.config.jobs.append(name)
			self.currentjob = name
			# G
			self.recordWorker = RecordWorker(name, self.listbox.get(0,tk.END), self.config)
			#  Disable this frame and change capture button to show stop.
			self.captureButton.config(text="Stop", command=self.stopcapturerequest)
		else:
			return
	
	def stopcapturerequest(self):
		try:
			self.recordWorker.endall()
		except Exception,e :
			print str(e)
		self.captureButton.config(text="Capture", command=self.capturerequest)

	def joblist(self):
		menubutton = tk.Menubutton(self,text='Select Jobs',
							 relief=tk.RAISED, width=20, justify=tk.CENTER)

		menubutton.menu = tk.Menu(menubutton, tearoff=0)
		menubutton['menu'] = menubutton.menu
		for job in self.config.jobs:
			menubutton.menu.add_command(label=job, command=lambda val=job: self.updateCurrentJob(val))
		return menubutton

	def updateCurrentJob(self, job):
		self.currentjob = job
		self.jobLabel.config(text=job)

	#----------------------Position Cameras--------------------------------------
class SimpleHUD(tk.Frame):
	def __init__(self, parent, top, bg_imgpath,fg_imgpath,xoffset,xintr, yoffset, yintr):
		tk.Frame.__init__(self,parent)
		self.bg_imgpath =bg_imgpath
		self.fg_imgpath = fg_imgpath
		self.xoffset = xoffset
		self.yoffset = yoffset
		self.parent = parent
		self.top = top
		self.columnconfigure(0,weight=1)
		self.rowconfigure(0,weight=1)
		self.fgimg = Image.open(self.fg_imgpath)
		self.i = 0
		self.j=0
		self.xintr =xintr
		self.yintr = yintr
		self.height = self.top.winfo_reqheight()
		self.width = self.top.winfo_reqwidth()
		self.tempimg = Image.open( self.bg_imgpath)
		self.tempimg.paste(self.fgimg, (self.i, 40))
		self.image = ImageTk.PhotoImage(self.tempimg)
		self.display = tk.Canvas(self, bd=0, highlightthickness=0)
		self.display.create_image(0, 0, image=self.image, anchor=tk.NW, tags="IMG")
		self.display.grid(row=0, sticky=tk.W+tk.E+tk.N+tk.S)
		self.pack(fill=tk.BOTH, expand=1)
		
		# self.bind("<Configure>", self.resize)
		self.update = self.draw(self.bg_imgpath)
		self.top.after(500, self.update)

	def draw(self, bg_imgpath):
		self.i = (self.i+self.xintr)% 80
		self.j = (self.j+self.yintr)% 80
		self.tempimg = Image.open( self.bg_imgpath)
		self.tempimg.paste(self.fgimg, (self.xoffset +self.i,self.j+self.yoffset ))
		self.image = ImageTk.PhotoImage(self.tempimg)
		self.display.delete("IMG")
		self.display.create_image(0, 0, image=self.image, anchor=tk.NW, tags="IMG")
		self.top.after(500, lambda :self.draw(self.bg_imgpath))
		# self.event_generate("<Configure>")
		self.resize()

	def resize(self):
		size = (self.width, self.height)
		resized = self.tempimg.resize(size,Image.ANTIALIAS)
		self.image = ImageTk.PhotoImage(resized)
		self.display.delete("IMG")
		self.display.create_image(0, 0, image=self.image, anchor=tk.NW, tags="IMG")

class PitchHUD(SimpleHUD):
	def __init__(self, parent, top):
		SimpleHUD.__init__(self,parent,top, 'pitch_hud.png', 'pitch_cam.png',95,0,155,5)
class YawHUD(SimpleHUD):
	def __init__(self, parent, top):
		SimpleHUD.__init__(self,parent,top, 'yaw_hud.png', 'yaw_cam.png',120,5,133,0)


class CameraMenuGetCam(CameraDropDownMenu):
	#Todo: Implement camera object and its dictionary
	def __init__(self, parent, config):
		CameraDropDownMenu.__init__(self,parent, config)
		for cam in self.cameraList:
			check = tk.StringVar()
			self.menu.add_command(label=cam, command=lambda val=cam: self.setCameraName(val))

	def setCameraName(self, val):
		self.cameraName = val
		self.config(text=self.cameraName)


def positionCamera(config):
	tp = tk.Toplevel()
	tp.resizable(width=tk.FALSE, height=tk.FALSE)
	tp.title("Position Camera")

	tp.positionCameraFrame= PositionCameraFrame(tp,config)
	tp.positionCameraFrame.pack()
	tp.protocol("WM_DELETE_WINDOW", tp.positionCameraFrame.closeParent)

class PositionCameraFrame(tk.Frame):
	def __init__(self,parent, config):
		tk.Frame.__init__(self,parent)
		self.parent = parent
		self.config = config
		#Pitch Display
		self.pitchlabel =tk.Label(self, text="Pitch = 20")
		self.pitchlabel.grid(row=2, column=0,padx=140,sticky=tk.W)
		framePitch = tk.Frame(self)
		framePitch.grid(row=4,column=0, padx=80) 
		self.pitchhud = PitchHUD(framePitch,self.parent)

		#Yaw Display
		self.yawlabel =tk.Label(self, text="Yaw: 20")
		self.yawlabel.grid(row=2, column=1,padx=70,sticky=tk.W)
		frameYaw = tk.Frame(self)
		frameYaw.grid(row=4,column=1) 
		self.yawhud = YawHUD(frameYaw, self.parent)

		self.cameraMenu = tk.Menubutton(self,text='Select Cameras',
							 relief=tk.RAISED, width=15, justify=tk.CENTER)
		self.cameraMenu.grid(row=1, column=0,padx=100,sticky=tk.W)
		self.menu = tk.Menu(self.cameraMenu, tearoff=0)
		self.cameraMenu['menu'] = self.menu
		self.selectedCam = ''
		for cam in self.config.cameras.cameras:
			self.menu.add_command(label=cam, command=lambda val=cam: self.getpose(val))

	def closeParent(self):
		self.stop()
		self.parent.destroy()
	def stop(self):
		try:
			self.poseReader.endall()
		except:
			print "Could not stop"

	def getpose(self, cam):
		if self.selectedCam != cam:
			self.selectedCam = cam
			self.poseReader = PoseReader(self.config, cam)
			#start redrawing
			self.redrawWorker = threading.Thread(target=self.redraw)
			time.sleep(10)
			self.redrawWorker.start()
			print  "Check"
	def redraw(self):
		while (self.poseReader.poll() == None):
			p,y = self.poseReader.read()

			msg = "Pitch: {:1.2f}".format(p)
			self.pitchlabel.config(text=msg)

			msg ="Yaw: {:1.2f}".format(y)
			self.yawlabel.config(text=msg)
			self.update()
			time.sleep(.1)
#--------------Change Cameras---------------------------
class CameraTypeDropDownMenu(tk.Menubutton):
	def __init__(self, parent):
		tk.Menubutton.__init__(self, parent,text='Camera Type',
							 relief=tk.RAISED, width=15,justify=tk.CENTER)

		self.menu = tk.Menu(self, tearoff=0)
		self['menu'] = self.menu
		# ToDo: Read from config file. Hard code for now.
		self.cameraTypeList = CameraType
		self.cameraType = str()

		for camtype in self.cameraTypeList:
			self.menu.add_command(label=camtype.value, command=lambda val=camtype.value: self.setCameraType(val))

	def setCameraType(self, val):
		self.cameraType = val
		self.config(text=self.cameraType)



def changeCameras(config):
	tp = tk.Toplevel()
	tp.resizable(width=tk.FALSE, height=tk.FALSE)
	tp.title("Change Cameras")
	ChangeCameraFrame(tp, config).pack()


	

class ChangeCameraFrame(tk.Frame):
	def __init__(self,parent, config):
		tk.Frame.__init__(self, parent)
		self.config = config
		editFrame = tk.Frame(self)

		editFrame.grid(row=1,column=2)
		tk.Label(editFrame, text="Camera Name").grid(row=0,column=0)
		self.cameraNameEntry = tk.Entry(editFrame, width=20, textvariable=tk.StringVar())
		self.cameraNameEntry.grid(row=1,column=0)
		self.cameraTypeDropDownMenu = CameraTypeDropDownMenu(editFrame)
		self.cameraTypeDropDownMenu.grid(row=2,column=0, pady=10)
		# #ToDo: Global object for camera type.
		# cameratypeFrame = tk.Frame(editFrame)
		# cameratypeFrame.grid(row=3, column=0, pady=10)
		# tp.cameraType = tk.StringVar()
		# for name,val in cameraType:
		# 	r = tk.Radiobutton(cameratypeFrame, text=name, value=val, command=lambda v=val: setattr(tp, 'cameraType', v))
		# 	r.pack(anchor=tk.W)
		self.config = config
		tk.Label(editFrame, text="Camera Address/Port").grid(row=4,column=0)
		self.cameraAddressEntry = tk.Entry(editFrame, width=20, textvariable=tk.StringVar())
		self.cameraAddressEntry.grid(row=6,column=0)

		ButtonFactory(self, "Add", lambda: self.addCamera()).grid(row=7, column=2)


		self.cameraListFrame = CameraListFrame(self, lambda :self.deleteCam() )
		# Get list of camera. Stubbed for now
		tk.Label(self, text="All Cameras").grid(row=0,column=0)

		self.updateCameraList()

		ttk.Separator(self, orient=tk.VERTICAL).grid(row=1, column=1, rowspan=8, sticky=tk.NS, padx=5)
	def deleteCam(self):
		val = self.cameraListFrame.listbox.get(self.cameraListFrame.listbox.curselection())
		# val = self.cameraListFrame.listbox.get(key)
		del self.config.cameras.cameras[val]

	def addCamera(self):
		name = self.cameraNameEntry.get()
		type_t = self.cameraTypeDropDownMenu.cameraType
		address = self.cameraAddressEntry.get()
		if name in self.config.cameras.cameras.iteritems():
			self.config.cameras.cameras[name].type = type_t
			self.config.cameras.cameras[name].address = address
		else:
			camera = Camera(name,type_t,address)
			self.config.cameras.cameras[name] = camera
		self.updateCameraList()

	def updateCameraList(self):
		self.cameraListFrame.listbox.delete(0, tk.END)
		for cam in self.config.cameras.cameras:
			self.cameraListFrame.listbox.insert(tk.END, cam )
		self.cameraListFrame.grid(row=1,column=0)
		self.cameraListFrame.listbox.bind("<<ListboxSelect>>",   lambda x: self.updateEntry())

	def updateEntry(self):
		# A marker and camera name must ever be the same.
		# Entry insert is unique
		self.cameraNameEntry.delete(0,'end')
		value = self.cameraListFrame.listbox.get(self.cameraListFrame.listbox.curselection())
		self.cameraNameEntry.insert(0,value)

		address = self.config.cameras.cameras[value].address
		self.cameraAddressEntry.delete(0,'end')
		self.cameraAddressEntry.insert(0,address)
#-----------Marker size ----------
def setMarkerSize(config):
	tp = tk.Toplevel()
	tp.resizable(width=tk.FALSE, height=tk.FALSE)
	tp.title("Marker Size")
	# Get Current Marker Size
	MarkerSizeFrame(tp,config).pack()

class MarkerSizeFrame(tk.Frame):
	def __init__(self, parent, config):
		tk.Frame.__init__(self,parent)
		self.parent = parent
		txt = "Current marker side length is " + str(config.markersize) + ". Enter new length"
		tk.Label(self, text= txt).grid(row=1,column=0,columnspan=2)
		length = tk.IntVar()
		self.config = config
		self.markersizeEntry = tk.Entry(self,textvariable=length)
		self.markersizeEntry.grid(row=2,column=0, columnspan=2)
		ButtonFactory(self,"Save", self.updateMarkerSize).grid(row=3,column=0)
		ButtonFactory(self,"Dismiss", self.parent.destroy).grid(row=3,column=1)
	def updateMarkerSize(self):
		self.config.markersize = self.markersizeEntry.get()
#--------------Camera Pose-------------------
class OriginSelectFrame(tk.Frame):
	def __init__(self, parent,config):
		tk.Frame.__init__(self, parent)
		self.config = config
		self.origin = tk.StringVar()
		self.mb = tk.Menubutton(self, text='Origin Type',
							 relief=tk.RAISED, width=15, justify=tk.CENTER)
		self.mb.grid(row=1, column=0)
		self.mb.menu = tk.Menu(self.mb, tearoff=0)
		self.mb['menu'] = self.mb.menu
		# ToDo: Read from config file. Hard code for now.
		self.originTypes = OriginType
		# self.setOriginType(Origin.Marker.value)
		for origintype in self.originTypes:
			self.mb.menu.add_command(label=origintype.value, command= lambda val=origintype.value: self.setOriginType(val))

	def setOriginType(self, val):
		self.originType = val

		if hasattr(self,'originWidget'):
				self.originWidget.destroy()
		if self.originType == 'Marker':
			# print 'Marker'
			self.origin.set("")
			self.originWidget = tk.Entry(self,width=15, textvariable=self.origin)
		else:
			self.originWidget = CameraDropDownMenu(self,self.config)

			for camkey in self.config.cameras.cameras:
				# cam = self.config.cameras.cameras[key]
				check = tk.StringVar()
				self.originWidget.menu.add_command(label=camkey, command=lambda val=camkey: self.updateCameraMenu(val))
			# print 'Camera'
		self.originWidget.grid(row=2,column=0, pady=10)
		self.mb.config(text=self.originType)
	def updateCameraMenu(self,cam):
		self.origin.set(cam)
		self.originWidget.config(text=cam)

class cameraPoseFrame(tk.Frame):
	def __init__(self,parent,config):
		tk.Frame.__init__(self,parent)
		self.jobname =''
		self.config = config
		self.unknownCamera = tk.StringVar()
		self.knownFrame = tk.Frame(self)
		self.knownFrame.grid(row=1,column=0)
		tk.Label(self.knownFrame, text="Known Point").grid(row=1,column=0)
		self.originSelectFrame = OriginSelectFrame(self.knownFrame,self.config)
		self.originSelectFrame.grid(row=2,column=0)
		self.knownPoint = self.originSelectFrame.origin
		print self.knownPoint.get()

		var = tk.IntVar()
		tk.Label(self, text="Unknown Camera").grid(row=3,column=0)
		self.originCheck = tk.Checkbutton (self, variable=var, text="Use as Origin", command=self.useAsOrigin )
		self.originCheck.var = var
		self.originCheck.grid(row =4,column=0)


		self.unknownCameraMenu = CameraDropDownMenu(self,self.config)
		for camkey in self.config.cameras.cameras:
				check = tk.StringVar()
				self.unknownCameraMenu.menu.add_command(label=camkey, command=lambda val=camkey: self.updateUnknownCamera(val))
		self.unknownCameraMenu.grid(row=6,column=0)
		frame = tk.Frame(self)
		frame.grid(row=7,column=0, pady=10)
		self.joblist(frame).grid(row=1)
		ButtonFactory2(frame, "Estimate", self.computeCameraPose,justify=tk.CENTER, pady=10).grid(row=2,column=0)
	
	def joblist(self, frame):
			menubutton = tk.Menubutton(frame,text='Select Jobs',
								 relief=tk.RAISED, width=20, justify=tk.CENTER)

			menubutton.menu = tk.Menu(menubutton, tearoff=0)
			menubutton['menu'] = menubutton.menu
			for job in self.config.jobs:
				menubutton.menu.add_command(label=job, command=lambda val=job: self.updateCurrentJob(val))
			return menubutton
	def updateCurrentJob(self, val):
		self.jobname = val
		print("current jobname: \"", self.jobname,"\"")
	def useAsOrigin(self):
		if self.originCheck.var.get():
			self.knownFrame.grid_forget()
		else:
			self.knownFrame.grid(row=1,column=0)

	def updateKnownCamera(self,val):
		self.knownCamera.set(val)
		self.knownCameraMenu.config(text=val)
		print "known "+ val

	def updateUnknownCamera(self,val):
		self.unknownCamera.set(val)
		self.unknownCameraMenu.config(text=val)
		print "unknown " + val

	def computeCameraPose(self):
		if self.originCheck.var.get():
			pose = [0, 0, 0, 0, 0, 0, 1, 1]
			self.config.cameras.cameras[self.unknownCamera.get()].pose = pose
		else:
			if self.originSelectFrame.originType == OriginType.Marker.value:
				camposeworker = CameraPoseWorker(self.config, None,self.unknownCamera.get(),int(self.knownPoint.get()))
			else:
				camposeworker = CameraPoseWorker(self.config, self.knownPoint.get(),self.unknownCamera.get(), None, self.jobname)
			camposeworker.worker.join()
			pose = camposeworker.estimatedpose
		resultsFrame = tk.Frame(self)
		resultsFrame.grid(row=10)
		tk.Label(resultsFrame, text="Camera Pose: ").pack(anchor=tk.N)

		for data in pose:
				label = tk.Label(resultsFrame, text=str(data))
				label.pack(anchor=tk.N)

def cameraPose(config):
	tp = tk.Toplevel()
	tp.resizable(width=tk.FALSE, height=tk.FALSE)
	tp.title("Estimate Camera Pose")
	cameraPoseFrame(tp, config).grid(row=0,column=0)

#-------------Main Menu---------------------
class MainMenu(tk.Menu):
	def __init__(self, parent, config):
		tk.Menu.__init__(self, parent)
		self.config = config
		filemenu = tk.Menu(self, tearoff=0)
		filemenu.add_command(label="New project", command=self.newProject)
		filemenu.add_command(label="Open project", command=self.loadProject)
		filemenu.add_command(label="Save project", command=self.saveProject)
		filemenu.add_command(label="Save project as...", command=None)
		self.add_cascade(label="File", menu=filemenu)

		self.cameraViewMenu = tk.Menu(self, tearoff=0)

		self.add_cascade(label="View Camera", menu=self.cameraViewMenu )
		self.cameraViewFlag = 0
		self.cameraViewMenu.add_command(label="Toggle Viewer", command=self.toggleViewer)
		self.cameraViewMenu.add_separator()
		self.cameraMenu = tk.Menu(self, tearoff=0,postcommand=self.updateCameraMenu)
		self.cameraViewMenu.add_cascade(label="Cameras", menu=self.cameraMenu)
		self.cameraViewMenu.entryconfig("Cameras",state="disabled")

	def toggleViewer(self):
		if self.cameraViewFlag == 0:
			self.cameraViewFlag = 1
			self.cameraViewMenu.entryconfig("Cameras",state="normal")
		else:
			self.cameraViewFlag = 0
			self.cameraViewMenu.entryconfig("Cameras",state="disabled")

	def updateCameraMenu(self):
		self.cameraMenu.delete(0,tk.END)

		for cam in self.config.cameras.cameras:
			self.cameraMenu.add_radiobutton(label=cam, command=lambda val=cam: self.viewCamera(val))

		print 'Check CamViewMenu'
	def viewCamera(self,val):

		print 'View Camera!'
	def newProject(self):
		self.newprojectframe = tk.Toplevel()
		self.newprojectframe.resizable(width=tk.FALSE, height=tk.FALSE)
		self.newprojectframe.title("New Project")
		tk.Label(self.newprojectframe, text="Type Project Name",anchor=tk.W).grid(row=1)
		self.newprojectframe.nameEntry = tk.Entry(self.newprojectframe,width=20)
		self.newprojectframe.nameEntry.grid(row=2)
		frame = tk.Frame(self.newprojectframe)
		frame.grid(row=4)
		self.newprojectframe.locationLabel = tk.Label(frame, text="Location",justify=tk.LEFT,anchor=tk.W)
		self.newprojectframe.locationEntry = tk.Entry(frame, textvariable=tk.StringVar(), width=40)
		self.newprojectframe.locationEntry.grid(row=4,column=1)
		self.newprojectframe.locationLabel.grid(row=4)
		self.newprojectframe.locationButton = tk.Button(frame, text="...", command=self.requestDir)
		self.newprojectframe.locationButton.grid(row=4,column=2)

		frame = tk.Frame(self.newprojectframe)
		frame.grid(row=6)
		ButtonFactory(frame,"Save", self.newprojectStore).grid(row=0)
		ButtonFactory(frame,"Cancel", None).grid(row=0,column=1)
	def newprojectStore(self):
		self.config.dirname = self.newprojectframe.locationEntry.get() + "/" + self.newprojectframe.nameEntry.get()
		if os.path.exists(self.config.dirname):
		    pass
		else:
		    os.mkdir( self.config.dirname , 0755 )
		self.newprojectframe.destroy()
	def requestDir(self):
		dir_opt = options = {}
		options['title'] = 'Specify Project Location'
		options['initialdir'] = '$HOME'
		dirname = tkFileDialog.askdirectory(**dir_opt)
		# 
		if len(dirname) >0:
			try:
				self.newprojectframe.locationEntry.delete(0, 'end')
				self.newprojectframe.locationEntry.insert(0, dirname)
			except:
				pass
	def saveProject(self):
		configxml = self.config.configToXML()
		xmlfilename = self.config.dirname + "/config.xml"
		output_file = open( xmlfilename, 'w' )
		output_file.write( '<?xml version="1.0"?>' )
		output_file.write( ElementTree.tostring( configxml ) )
		output_file.close()

	def loadProject(self):
		file_opt = options = {}
		options['title'] = 'Please select a project xml file'
		options['initialdir'] = '$HOME'
		options['filetypes'] = [('xml files', '.xml')]
		filename = tkFileDialog.askopenfilename(**file_opt)
		if len( filename ) > 0:
			self.config.init()
			self.config.readFile(filename)

			print "Dirname is %s" % self.config.dirname
		# Read xml file.

def calibrate(config):
	tp = tk.Toplevel()
	tp.resizable(width=tk.FALSE, height=tk.FALSE)
	tp.title("Upload Camera Calibration")
	CalibrateFrame(tp,config).grid()
class CalibrateFrame(tk.Frame):
	def __init__(self,parent, config):
		tk.Frame.__init__(self,parent)
		self.config = config
		self.cameraMenu = CameraMenuGetCam(self,config)
		self.cameraMenu.grid(row=1, column=0,padx=100,sticky=tk.W)

		
		tk.Label(self, text="Insert Calibration Resolution").grid(row=2, columnspan=4)
		tempFrame = tk.Frame(self)
		tempFrame.grid(row=3)
		tk.Label(tempFrame, text="x: ").grid(row=3, column=0)
		self.xEntry = tk.Entry(tempFrame, width=5)
		self.xEntry.grid(row=3, column=1)
		tk.Label(tempFrame, text="y: ").grid(row=3, column=2)
		self.yEntry = tk.Entry(tempFrame, width=5)
		self.yEntry.grid(row=3, column=3)


		ButtonFactory(self, "Upload calibration",self.requestDir ).grid(row=5)
	def requestDir(self):
		file_opt = options = {}
		options['title'] = 'Please select calibration xml file'
		options['initialdir'] = '$HOME'
		options['filetypes'] = [('xml files', '.xml')]
		filename = tkFileDialog.askopenfilename(**file_opt)
		if len( filename ) > 0:
			cameraName = self.cameraMenu.cameraName
			self.config.cameras.cameras[cameraName].calibration.readfromFile(filename)
			
			txt = []
			calibration = self.config.cameras.cameras[cameraName].calibration
			calibration.calib['x'] = int(self.xEntry.get())
			calibration.calib['y'] = int(self.yEntry.get())
			tempFrame = tk.Frame(self)
			tempFrame.grid(row=6)
			for tag in calibration.tags:
				msg = tag + ": " + str(calibration.calib[tag])
				txt.append(msg)
				tk.Label(tempFrame, text=msg).pack()