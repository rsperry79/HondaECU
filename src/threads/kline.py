import time
import os
import wx
from threading import Thread
from pydispatch import dispatcher
import numpy as np

import pyftdi

from eculib import KlineAdapter
from eculib.honda import *

class KlineWorker(Thread):

	def __init__(self, parent):
		self.parent = parent
		self.__clear_data()
		dispatcher.connect(self.DeviceHandler, signal="FTDIDevice", sender=dispatcher.Any)
		dispatcher.connect(self.ErrorPanelHandler, signal="ErrorPanel", sender=dispatcher.Any)
		dispatcher.connect(self.DatalogPanelHandler, signal="DatalogPanel", sender=dispatcher.Any)
		dispatcher.connect(self.ReadPanelHandler, signal="ReadPanel", sender=dispatcher.Any)
		dispatcher.connect(self.WritePanelHandler, signal="WritePanel", sender=dispatcher.Any)
		dispatcher.connect(self.HRCSettingsPanelHandler, signal="HRCSettingsPanel", sender=dispatcher.Any)
		Thread.__init__(self)

	def __cleanup(self):
		if self.ecu:
			try:
				self.ecu.dev.close()
			except pyftdi.ftdi.FtdiError as e:
				pass
			del self.ecu
		self.__clear_data()

	def __clear_data(self):
		self.ecu = None
		self.ready = False
		self.sendpassword = False
		self.sendwriteinit = False
		self.sendrecoverinint = False
		self.hrcmode = None
		self.senderase = False
		self.readinfo = None
		self.writeinfo = None
		self.state = ECUSTATE.UNKNOWN
		self.reset_state()

	def reset_state(self):
		self.ecmid = bytearray()
		self.errorcodes = {}
		self.update_errors = False
		self.clear_codes = False
		self.flashcount = -1
		self.dtccount = -1
		self.update_tables = False
		self.tables = None
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="ecmid", value=bytes(self.ecmid))
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="flashcount", value=self.flashcount)

	def HRCSettingsPanelHandler(self, mode, data):
		self.hrcmode = (mode, data)

	def WritePanelHandler(self, data, offset):
		self.writeinfo = [data,offset,None]

	def ReadPanelHandler(self, data, offset, passwd):
		if self.state != ECUSTATE.SECURE:
			self.sendpassword = True
		self.readinfo = [data,offset,None, passwd]

	def DatalogPanelHandler(self, action):
		if action == "data.on":
			self.update_tables = True
		elif action == "data.off":
			self.update_tables = False

	def ErrorPanelHandler(self, action):
		if action == "dtc.clear":
			self.clear_codes = True
		elif action == "dtc.on":
			self.update_errors = True
		elif action == "dtc.off":
			self.update_errors = False

	def DeviceHandler(self, action, device, config):
		if action == "interrupt":
			raise Exception()
		elif action == "deactivate":
			if self.ecu:
				self.__cleanup()
		elif action == "activate":
			self.__clear_data()
			self.ecu = HondaECU(KlineAdapter(config))
			self.ready = True

	def read_flash(self):
		readsize = 12
		location = self.readinfo[1]
		binfile = self.readinfo[0]
		status = "bad"
		with open(binfile, "wb") as fbin:
			t = time.time()
			size = location
			rate = 0
			while not self.readinfo is None:
				status, data = self.ecu._read_flash_bytes(location, readsize)
				if status:
					fbin.write(data)
					fbin.flush()
					location += readsize
					n = time.time()
					wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="read.progress", value=(-1,"%.02fKB @ %s" % (location/1024.0, "%.02fB/s" % (rate) if rate > 0 else "---")))
					if n-t > 1:
						rate = (location-size)/(n-t)
						t = n
						size = location
				else:
					readsize -= 1
					if readsize < 1:
						break
			if self.ecu.dev.kline():
				wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="read.progress", value=(-1,"%.02fKB @ %s" % (location/1024.0, "%.02fB/s" % (rate) if rate > 0 else "---")))
			else:
				return "interrupted"
		with open(binfile, "rb") as fbin:
			nbyts = os.path.getsize(binfile)
			if nbyts > 0:
				byts = bytearray(fbin.read(nbyts))
				_, status, _ = do_validation(byts, nbyts)
			return status

	def write_flash(self, byts, offset=0):
		ossize = len(byts)
		offseti = int(offset/16)
		writesize=128
		z = int(writesize/16)
		maxi = int(ossize/writesize)
		i = 0
		w = 0
		t = time.time()
		rate = 0
		size = 0
		while not self.writeinfo is None and i < maxi:
			w = (i*writesize)
			bytstart = [s for s in struct.pack(">H",offseti+(z*i))]
			if i+1 == maxi:
				bytend = [s for s in struct.pack(">H",0)]
			else:
				bytend = [s for s in struct.pack(">H",offseti+(z*(i+1)))]
			d = list(byts[((i+0)*writesize):((i+1)*writesize)])
			x = bytstart + d + bytend
			c1 = checksum8bit(x)
			c2 = checksum8bitHonda(x)
			x = [0x01, 0x06] + x + [c1, c2]
			info = self.ecu.send_command([0x7e], x)
			if info is not None:
				if ord(info[1]) != 5:
					wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "interrupted"))
					return 1
			else:
				if i == 0:
					if writesize == 128:
						writesize = 64
						z = int(writesize/16)
						maxi = int(ossize/writesize)
						continue
					else:
						wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "failed"))
						return 2
				else:
					wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "interrupted"))
					return 3
			n = time.time()
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(i/maxi*100,"%.02fKB of %.02fKB @ %s" % (w/1024.0, ossize/1024.0, "%.02fB/s" % (rate) if rate > 0 else "---")))
			if n-t > 1:
				rate = (w-size)/(n-t)
				t = n
				size = w
			i += 1
			if i % 2 == 0:
				if writesize == 64:
					self.ecu.send_command([0x7e], [0x01, 0x07])
					time.sleep(.200)
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="progress", value=(i/maxi*100,"%.02fKB of %.02fKB @ %s" % ((w-offset)/1024.0, ossize/1024.0, "%.02fB/s" % (rate) if rate > 0 else "---")))
		return 0

	def do_init_write(self, recover=False):
		self.do_update_state()
		if recover:
			self.ecu.do_init_recover()
			self.ecu.send_command([0x72],[0x00, 0xf1])
			time.sleep(1)
			self.ecu.send_command([0x27],[0x00, 0x01, 0x00])
		else:
			self.ecu.do_init_write()
		self.do_update_state()
		time.sleep(.100)

	def do_erase(self, wait=11):
		self.do_update_state()
		ret = 1
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="erase", value=None)
		self.ecu.get_write_status()
		for i in range(wait):
			w = wait-i
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(w/wait*100, "waiting for %d seconds" % (w)))
			time.sleep(1)
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "waiting for %d seconds" % (w)))
		if self.ecu.do_erase():
			time.sleep(2)
			e = 0
			while True:
				wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(np.clip(e/35*100,0,100), "erasing ecu"))
				time.sleep(.1)
				e += 1
				info = self.ecu.send_command([0x7e], [0x01, 0x05])
				if info:
					if info[2][1] == 0x00:
						self.ecu.get_write_status()
						ret = 0
						break
					elif info[2][1] == 0xfa:
						wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "erase block error"))
						ret = 2
						break
				else:
					break
		else:
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.progress", value=(0, "erase failed"))
		self.do_update_state()
		return ret

	def do_write(self):
		self.do_update_state()
		ret = 1
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write", value=None)
		if self.write_flash(self.writeinfo[0], offset=self.writeinfo[1]) == 0:
			self.writeinfo[2] = "good" if self.ecu.do_post_write() else "bad"
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="write.result", value=self.writeinfo[2])
			ret = 0
		self.do_update_state()
		return ret

	def do_read(self):
		self.do_update_state()
		ret = 1
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="read", value=None)
		self.readinfo[2] = self.read_flash()
		if self.readinfo[2] == "interrupted":
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="read.progress", value=(0, "interrupted"))
		else:
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="read.result", value=self.readinfo[2])
			ret = 0
		self.do_update_state()
		return ret

	def do_get_ecmid(self):
		ret = 1
		info = self.ecu.send_command([0x72], [0x71, 0x00])
		if info:
			self.ecmid = info[2][2:7]
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="ecmid", value=bytes(self.ecmid))
			ret = 0
		return ret

	def do_get_flashcount(self):
		ret = 1
		info = self.ecu.send_command([0x7d], [0x01, 0x01, 0x03])
		if info:
			self.flashcount = int(info[2][4])
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="flashcount", value=self.flashcount)
			ret = 0
		return ret

	def do_clear_codes(self):
		ret = 1
		while self.clear_codes:
			info = self.ecu.send_command([0x72],[0x60, 0x03])
			if info:
				if info[2][1] == 0x00:
					ret = 0
					self.dtccount = -1
					self.errorcodes = {}
					self.clear_codes = False
			else:
				self.dtccount = -1
				self.errorcodes = {}
				self.clear_codes = False
		return ret

	def do_get_dtcs(self):
		errorcodes = {}
		for type in [0x74,0x73]:
			errorcodes[hex(type)] = []
			for i in range(1,0x0c):
				info = self.ecu.send_command([0x72],[type, i])
				if info:
					for j in [3,5,7]:
						if info[2][j] != 0:
							errorcodes[hex(type)].append("%02d-%02d" % (info[2][j],info[2][j+1]))
					if info[2] == 0:
						return 1
				else:
					return 1
		dtccount = sum([len(c) for c in errorcodes.values()])
		if self.dtccount != dtccount:
			self.dtccount = dtccount
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="dtccount", value=self.dtccount)
		if self.errorcodes != errorcodes:
			self.errorcodes = errorcodes
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="dtc", value=self.errorcodes)
		return 0

	def do_probe_tables(self):
		tables = self.ecu.probe_tables()
		if len(tables) > 0:
			self.tables = tables
			tables = " ".join([hex(x) for x in self.tables.keys()])
			for t, d in self.tables.items():
				wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="data", value=(t,d[0],d[1]))
			return 0
		else:
			return 1

	def do_update_tables(self):
		for t in self.tables:
			info = self.ecu.send_command([0x72], [0x71, t])
			if info:
				if info[3] > 2:
					self.tables[t] = [info[3],info[2]]
					wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="data", value=(t,info[3],info[2]))
				else:
					return 1
			else:
				return 1
		return 0

	def do_basic_tasks(self):
		ret = 0
		if not self.ecmid:
			ret += self.do_get_ecmid()
		return ret

	def do_idle_tasks(self):
		ret = self.do_basic_tasks()
		if self.flashcount < 0:
			ret += self.do_get_flashcount()
		if self.clear_codes:
			ret += self.do_clear_codes()
		if self.update_errors or self.dtccount < 0:
			ret += self.do_get_dtcs()
		if not self.tables:
			ret += self.do_probe_tables()
		elif self.update_tables:
			ret += self.do_update_tables()
		return ret

	def do_update_state(self):
		state = self.ecu.detect_ecu_state()
		if state != self.state:
			self.state = state
			wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="state", value=self.state)
			if self.state == ECUSTATE.OFF:
				self.reset_state()

	def do_password(self):
		p1 = self.ecu.send_command([0x27],[0xe0] + self.readinfo[3][:7])
		p2 = self.ecu.send_command([0x27],[0xe0] + self.readinfo[3][7:])
		passok = (p1 != None) and (p2 != None)
		wx.CallAfter(dispatcher.send, signal="KlineWorker", sender=self, info="password", value=passok)

	def write_helper(self, init=False, recover=False, nodiag=False):
		ret = 1
		if nodiag or self.ecu.diag():
			if init:
				self.do_init_write(recover=recover)
				time.sleep(.100)
			if self.do_erase() == 0:
				self.do_write()
				ret = 0
			self.writeinfo = None
		return ret

	def read_helper(self):
		ret = self.do_read()
		self.readinfo = None
		return ret

	def do_on_power(self):
		if self.sendpassword:
			self.do_password()
			self.sendpassword = False

	def do_connected(self):
		if self.state == ECUSTATE.OK:
			if self.writeinfo is not None:
				return self.write_helper(init=True)
			else:
				return self.do_idle_tasks()
		elif self.state == ECUSTATE.RECOVER_OLD:
			self.do_basic_tasks()
			if self.writeinfo is not None:
				return self.write_helper(init=True)
		elif self.state == ECUSTATE.RECOVER_NEW:
			self.do_basic_tasks()
			if self.writeinfo is not None:
				return self.write_helper(init=True, recover=True)
		return False

	def do_exceptions(self):
		ret = 1
		if self.state == ECUSTATE.SECURE:
			if self.readinfo is not None:
				ret = self.read_helper()
		elif self.state == ECUSTATE.FLASH:
			if self.writeinfo is not None:
				return self.write_helper(nodiag=True)
		else:
			pass
		return ret

	def run(self):
		while self.parent.run:
			if not self.ready:
				time.sleep(.001)
			else:
				try:
					if self.state in [ECUSTATE.OFF, ECUSTATE.UNKNOWN]:
						self.do_update_state()
						if self.state not in [ECUSTATE.OFF, ECUSTATE.UNKNOWN]:
							self.do_on_power()
						else:
							self.ecu.init()

					else:
						if self.ecu.diag():
							if self.do_connected() > 0:
								self.do_update_state()
						else:
							if self.do_exceptions() > 0:
								self.do_update_state()
				except AttributeError:
					pass
				except OSError:
					pass
