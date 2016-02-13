
import Tkinter as tk
import ttk
import tkFileDialog
from PIL import Image, ImageTk
from menus import MainMenu
from menus import ViewerFrame
from menus import ButtonFactory
from menus import trackerWidget
from menus import positionCamera
from menus import changeCameras
from menus import setMarkerSize
from menus import cameraPose
from menus import calibrate
# from processes import loadProject
# from processes import saveProject
from processes import ProjectConfig
from processes import Camera #Delete after testing
from processes import Cameras
class MoreT2(tk.Frame):

	def __init__(self, parent, *args, **kwargs):
		tk.Frame.__init__(self, parent, *args, **kwargs)
		self.win = parent
		self.win.wm_title("MoRe-T2")
		# self.win.resizable(width=FALSE, height=FALSE)
		self.projectConfig = ProjectConfig()
		self.mainMenu = MainMenu(self,self.projectConfig)
		parent.config(menu=self.mainMenu)
		# self.projectConfig.markersize = 100
		# self.projectConfig.cameras = Cameras()
		# self.projectConfig.cameras.cameras["Front"] = Camera()
		self.displayFrame = ViewerFrame(self,self.projectConfig)
		self.displayFrame.grid(row=0, column=0, columnspan=3,sticky=tk.NSEW)

		# self.projectConfig.viewerCanvas = ViewerCanvas(displayFrame, self.projectConfig,width=600, height=450, bg = 'black')
		# # pack the canvas into a frame/form
		# self.projectConfig.viewerCanvas.pack(fill=tk.BOTH, expand=tk.YES)
		
		# imagelabel = Label(displayFrame, image=self.image)
		# imagelabel.image = self.image
		# imagelabel.grid(row=0, column=0)

		controlFrame = tk.Frame(self)
		controlFrame.grid(row=1, column=1)
		frame1 = tk.Frame(controlFrame)
		frame1.grid(row=0,column=0, sticky=tk.W)
		# Replace with dictionary
		# key, value in d.iteritems()
		rowButtons =  {	"Track": lambda: trackerWidget(self.projectConfig), 
						"Orient Camera": lambda: positionCamera(self.projectConfig),
						"Camera Pose":lambda:cameraPose(self.projectConfig)}
		for txt, fn in rowButtons.iteritems():
			b=ButtonFactory(frame1, txt, fn)
			b.pack(side=tk.LEFT,fill=tk.BOTH, expand=1)
		frame2 = tk.Frame(controlFrame)
		frame2.grid(row=1,column=0, sticky=tk.W)
		rowButtons =  {	"Change Camera":lambda: changeCameras(self.projectConfig),
						"Calibrate Camera": lambda: calibrate(self.projectConfig),
						"Marker Size": lambda: setMarkerSize(self.projectConfig),
						}
		for txt, fn in rowButtons.iteritems():
			b=ButtonFactory(frame2, txt, fn)
			b.pack(side=tk.LEFT,fill=tk.BOTH, expand=1)
		self.configure(background='grey')
		self.columnconfigure(0, weight=1)
		self.columnconfigure(2, weight=1)
		self.rowconfigure(0, weight=1)
		# self.rowconfigure(1, weight=0)
		self.win.lift()
		self.win.call('wm', 'attributes', '.', '-topmost', True)
		self.win.after_idle(self.win.call, 'wm', 'attributes', '.', '-topmost', False)
		self.pack(side="top", fill="both", expand=True)
		self.win.update()

if __name__ == "__main__":

	root = tk.Tk()
	MoreT2(root)
	root.mainloop()

# Printing out all attributes of config.
# 	an = Animal()
# attrs = vars(an)
# # {'kids': 0, 'name': 'Dog', 'color': 'Spotted', 'age': 10, 'legs': 2, 'smell': 'Alot'}
# # now dump this in some way or another
# print ', '.join("%s: %s" % item for item in attrs.items())
