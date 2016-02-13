
import Tkinter as tk
import ttk
import subprocess as sp
import signal
import os, sys, pipes
import time
import struct
import atexit

class TriggerGen(tk.Frame):

	def __init__(self, parent, *args, **kwargs):
		tk.Frame.__init__(self, parent, *args, **kwargs)
		self.win = parent

		self.timeLabel = tk.Label(self, text="N/A")
		self.timeLabel.grid(row=2)

		frame1 = tk.Frame(self)
		frame1.grid(row=3)
		self.getBtn = tk.Button(frame1,justify=tk.LEFT,anchor=tk.W,  width=5,height=1,text="Get", command=self.getTime)
		self.getBtn.grid(row=3, column=0)
		self.logBtn = tk.Button(frame1,justify=tk.LEFT,anchor=tk.W,  width=5,height=1,text="Log", command=self.logData)
		self.logBtn.grid(row=3, column=1)
		self.playPauseBtn = tk.Button(frame1,justify=tk.LEFT,anchor=tk.W,  width=5,height=1,text="P/P", command=self.playpause)
		self.playPauseBtn.grid(row=3, column=2)
		self.fastforwardBtn = tk.Button(frame1,justify=tk.LEFT,anchor=tk.W,  width=5,height=1,text="F/Fwd", command=self.fastforward)
		self.fastforwardBtn.grid(row=3, column=3)

		self.time = -1
		tk.Label(self, text="Capture Prefix:").grid(row=4)
		self.prefixEntry = tk.Entry(self,width=20, textvariable=tk.StringVar() )
		self.prefixEntry.grid(row=5)

		frame2 = tk.Frame(self)
		frame2.grid(row=6)
		cameralist =  {	"PamCam1", "PamCam2", "PamCam3", "PamCam4", "ACam5", "ACam6", "ACam7"}
		frame21 = tk.Frame(frame2)
		frame21.grid(row=1,column=0)

		tk.Label(frame2, text="Camera List:").grid(row=0,column=0)
		self.camera = tk.StringVar()
		for cam in cameralist:
			r = tk.Radiobutton(frame21, text=cam, variable=self.camera, value=cam)
			r.pack(anchor = tk.W)

		tasklist =  {	"sideslope", "curb", "turn", "door", "side", "back", "rotate"}
		frame22 = tk.Frame(frame2)
		frame22.grid(row=1,column=1)

		tk.Label(frame2, text="Task List:").grid(row=0,column=1)
		self.task = tk.StringVar()
		for task in tasklist:
			r = tk.Radiobutton(frame22, text=task, variable=self.task, value=task)
			r.pack(anchor = tk.W)

		frame1 = tk.Frame(self)
		frame1.grid(row=9)
		self.startstopBtn = tk.Button(frame1,justify=tk.LEFT,anchor=tk.W,  width=5,height=1,text="Start", command=self.startstop)
		self.startstopBtn.grid(row=3, column=2)

		atexit.register(self.exit)

		self.win.lift()
		self.win.call('wm', 'attributes', '.', '-topmost', True)
		self.win.after_idle(self.win.call, 'wm', 'attributes', '.', '-topmost', False)
		self.pack(side="top", fill="both", expand=True)
		self.win.update()
	def exit(self):
		os.killpg(self.proc.pid, signal.SIGTERM)
		self.FNULL.close()
		self.pipein.close()
	def fastforward(self):
		if self.proc.poll() ==None:
			self.proc.stdin.write('f\n')
	def playpause(self):
		if self.proc.poll() ==None:
			self.proc.stdin.write('p\n')
	def logData(self):
		filepath = self.prefixEntry.get() + "/" + self.task.get() + ".txt"
		if filepath:
			fp = open(filepath, 'a')
			fp.write(str(self.time) + '\n')
			fp.close()
	def getTime(self):
		if self.proc.poll() ==None:
		  	self.proc.stdin.write('c\n')
			print 'written!!'
			time.sleep(0.2)
			nums = self.pipein.read(8)
			t = struct.unpack('Q',nums)[0]
			print "time: {}".format(t)

			self.timeLabel.config(text=str(t))
			self.time = t
		else:
		  print "nothing available!"  # or just ignore that case

	def startstop(self):
		if ((hasattr(self, 'proc') and (self.proc.poll() !=None)) or (not hasattr(self, 'proc'))):
			#Process has stopped running
			vidpath = self.prefixEntry.get() + "/" + self.camera.get()
			cmd = ''
			if (vidpath):
				vidpath = pipes.quote(vidpath)
				cmd = os.path.dirname(os.path.realpath(__file__)) + "/../../moret2/bin/extra/triggergen"
				cmd += " " + vidpath
				self.FNULL = open(os.devnull, 'w')
				self.proc = sp.Popen(cmd,universal_newlines=True, shell=True, executable="/bin/bash",preexec_fn=os.setsid, stdin=sp.PIPE, stdout=self.FNULL)
				
				
				while not os.path.exists("/tmp/triggergen" ):
					pass

				self.pipein = open("/tmp/triggergen",'r')
				
				self.startstopBtn.config(text="Stop")
			return
		os.killpg(self.proc.pid, signal.SIGTERM)
		self.FNULL.close()
		self.pipein.close()
		self.startstopBtn.config(text="Start")
if __name__ == "__main__":
	root = tk.Tk()
	TriggerGen(root)
	root.mainloop()