import sys
import os
import time

from threading import Thread
from pylibftdi import Driver, FtdiError, LibraryMissingError
from pydispatch import dispatcher

import wx
import wx.lib.buttons as buttons

from appanels import *

class USBMonitor(Thread):

	def __init__(self, parent):
		self.parent = parent
		self.ftdi_devices = {}
		Thread.__init__(self)

	def run(self):
		while self.parent.run:
			time.sleep(.5)
			new_devices = {}
			try:
				for device in Driver().list_devices():
					vendor, product, serial = map(lambda x: x.decode('latin1'), device)
					new_devices[serial] = (vendor, product)
					if not serial in self.ftdi_devices:
						wx.CallAfter(dispatcher.send, signal="USBMonitor", sender=self, action="add", vendor=vendor, product=product, serial=serial)
				for serial in self.ftdi_devices:
					if not serial in new_devices:
						wx.CallAfter(dispatcher.send, signal="USBMonitor", sender=self, action="remove", vendor=self.ftdi_devices[serial][0], product=self.ftdi_devices[serial][1], serial=serial)
			except FtdiError as e:
				if sys.exc_info()[0] == LibraryMissingError:
					wx.LogError(str(e))
					break
			except LibraryMissingError as e:
				wx.LogError(str(e))
				break
			self.ftdi_devices = new_devices

class HondaECU_AppButton(buttons.ThemedGenBitmapTextButton):

	def __init__(self, appid, *args, **kwargs):
		self.appid = appid
		buttons.ThemedGenBitmapTextButton.__init__(self, *args,**kwargs)
		self.SetInitialSize((128,96))

	def DrawLabel(self, dc, width, height, dx=0, dy=0):
		bmp = self.bmpLabel
		if bmp is not None:
			if self.bmpDisabled and not self.IsEnabled():
				bmp = self.bmpDisabled
			if self.bmpFocus and self.hasFocus:
				bmp = self.bmpFocus
			if self.bmpSelected and not self.up:
				bmp = self.bmpSelected
			bw,bh = bmp.GetWidth(), bmp.GetHeight()
			if not self.up:
				dx = dy = self.labelDelta
			hasMask = bmp.GetMask() is not None
		else:
			bw = bh = 0
			hasMask = False

		dc.SetFont(self.GetFont())
		if self.IsEnabled():
			dc.SetTextForeground(self.GetForegroundColour())
		else:
			dc.SetTextForeground(wx.SystemSettings.GetColour(wx.SYS_COLOUR_GRAYTEXT))

		label = self.GetLabel()
		tw, th = dc.GetTextExtent(label)
		if not self.up:
			dx = dy = self.labelDelta

		if bmp is not None:
			dc.DrawBitmap(bmp, (width-bw)/2, (height-bh-th-4)/2, hasMask)
		dc.DrawText(label, (width-tw)/2, (height+bh-th+4)/2)

class HondaECU_ControlPanel(wx.Frame):

	def __init__(self):
		self.run = True
		self.active_ftdi_device = None
		self.ftdi_devices = {}

		if getattr(sys, 'frozen', False):
			self.basepath = sys._MEIPASS
		else:
			self.basepath = os.path.dirname(os.path.realpath(__file__))
		self.apps = {
			"read": {"label":"Read ECU", "icon":"pngs/controlpanel/download.png"},
			"tune": {"label":"Tune", "icon":"pngs/controlpanel/spanner.png"},
			"write": {"label":"Write ECU", "icon":"pngs/controlpanel/upload.png"},
			"data": {"label":"Data Logging", "icon":"pngs/controlpanel/chart.png"},
			"dtc": {"label":"Trouble Codes", "icon":"pngs/controlpanel/warning.png"},
			"info": {"label":"ECU Info", "icon":"pngs/controlpanel/info.png"},
		}
		self.appanels = {}

		wx.Frame.__init__(self, None, title="HondaECU :: Control Panel", style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER, size=(500,300))

		ib = wx.IconBundle()
		ib.AddIcon(os.path.join(self.basepath,"images/honda.ico"))
		self.SetIcons(ib)

		self.menubar = wx.MenuBar()
		self.SetMenuBar(self.menubar)
		fileMenu = wx.Menu()
		self.menubar.Append(fileMenu, '&File')
		self.devicesMenu = wx.Menu()
		fileMenu.Append(wx.ID_ANY, "Devices", self.devicesMenu)
		fileMenu.AppendSeparator()
		quitItem = wx.MenuItem(fileMenu, wx.ID_EXIT, '&Quit\tCtrl+Q')
		self.Bind(wx.EVT_MENU, self.OnClose, quitItem)
		fileMenu.Append(quitItem)
		helpMenu = wx.Menu()
		self.menubar.Append(helpMenu, '&Help')
		aboutItem = wx.MenuItem(helpMenu, wx.ID_ANY, 'About')
		self.Bind(wx.EVT_MENU, self.OnAbout, aboutItem)
		helpMenu.Append(aboutItem)

		self.statusbar = self.CreateStatusBar(1)
		self.statusbar.SetSize((-1, 28))
		self.statusbar.SetStatusStyles([wx.SB_SUNKEN])
		self.SetStatusBar(self.statusbar)

		self.wrappanel = wx.Panel(self)
		wrapsizer = wx.WrapSizer(wx.HORIZONTAL)
		self.appbuttons = {}
		for a,d in self.apps.items():
			icon = wx.Image(os.path.join(self.basepath, d["icon"]), wx.BITMAP_TYPE_ANY).ConvertToBitmap()
			self.appbuttons[a] = HondaECU_AppButton(a, self.wrappanel, wx.ID_ANY, icon, label=d["label"])
			wrapsizer.Add(self.appbuttons[a], 0)
			self.Bind(wx.EVT_BUTTON, self.OnAppButtonClicked, self.appbuttons[a])
		self.wrappanel.SetSizer(wrapsizer)
		mainsizer = wx.BoxSizer(wx.VERTICAL)
		mainsizer.Add(self.wrappanel,0,wx.ALIGN_CENTER)
		self.SetSizer(mainsizer)
		self.Bind(wx.EVT_CLOSE, self.OnClose)

		dispatcher.connect(self.USBMonitorHandler, signal="USBMonitor", sender=dispatcher.Any)
		dispatcher.connect(self.AppPanelHandler, signal="AppPanel", sender=dispatcher.Any)

		self.usbmonitor = USBMonitor(self)
		self.usbmonitor.start()

		self.Layout()
		mainsizer.Fit(self)
		self.Center()
		self.Show()

	def OnClose(self, event):
		self.run = False
		self.usbmonitor.join()
		for w in wx.GetTopLevelWindows():
			w.Destroy()

	def OnAbout(self, event):
		pass

	def OnAppButtonClicked(self, event):
		self.wrappanel.Disable()
		b = event.GetEventObject()
		if not b.appid in self.appanels:
			self.appanels[b.appid] = HondaECU_AppPanel(self, b.appid, self.apps[b.appid])
		self.appanels[b.appid].Raise()

	def USBMonitorHandler(self, action, vendor, product, serial):
		dirty = False
		if action == "add":
			if not serial in self.ftdi_devices:
				self.ftdi_devices[serial] = (vendor, product)
				dirty = True
		elif action =="remove":
			if serial in self.ftdi_devices:
				if serial == self.active_ftdi_device:
					dispatcher.send(signal="FTDIDevice", sender=self, action="deactivate", vendor=vendor, product=product, serial=serial)
					self.active_ftdi_device = None
				del self.ftdi_devices[serial]
				dirty = True
		if len(self.ftdi_devices) > 0:
			if not self.active_ftdi_device:
				self.active_ftdi_device = list(self.ftdi_devices.keys())[0]
				dispatcher.send(signal="FTDIDevice", sender=self, action="activate", vendor=vendor, product=product, serial=serial)
				dirty = True
		else:
				pass
		if dirty:
			for i in self.devicesMenu.GetMenuItems():
				self.devicesMenu.Remove(i)
			for s in self.ftdi_devices:
				rb = self.devicesMenu.AppendRadioItem(wx.ID_ANY, "%s : %s : %s" % (self.ftdi_devices[s][0], self.ftdi_devices[s][1], s))
				self.Bind(wx.EVT_MENU, self.OnDeviceSelected, rb)
			if self.active_ftdi_device:
				self.statusbar.SetStatusText("%s : %s : %s" % (self.ftdi_devices[self.active_ftdi_device][0], self.ftdi_devices[self.active_ftdi_device][1], self.active_ftdi_device), 0)
				self.devicesMenu.FindItemByPosition(list(self.ftdi_devices.keys()).index(self.active_ftdi_device)).Check()
			else:
				self.statusbar.SetStatusText("", 0)

	def OnDeviceSelected(self, event):
		s = list(self.ftdi_devices.keys())[[m.IsChecked() for m in event.GetEventObject().GetMenuItems()].index(True)]
		if s != self.active_ftdi_device:
			if self.active_ftdi_device != None:
				dispatcher.send(signal="FTDIDevice", sender=self, action="deactivate", vendor=self.ftdi_devices[self.active_ftdi_device], product=self.ftdi_devices[self.active_ftdi_device], serial=self.active_ftdi_device)
			self.active_ftdi_device = s
			dispatcher.send(signal="FTDIDevice", sender=self, action="activate", vendor=self.ftdi_devices[self.active_ftdi_device], product=self.ftdi_devices[self.active_ftdi_device], serial=self.active_ftdi_device)
			self.statusbar.SetStatusText("%s : %s : %s" % (self.ftdi_devices[self.active_ftdi_device][0], self.ftdi_devices[self.active_ftdi_device][1], self.active_ftdi_device), 0)

	def AppPanelHandler(self, appid, action):
		if action == "close":
			if appid in self.appanels:
				del self.appanels[appid]
				self.wrappanel.Enable()

if __name__ == '__main__':

	app = wx.App()
	gui = HondaECU_ControlPanel()
	app.MainLoop()