from __future__ import division
from pylibftdi import Device, FtdiError, Driver
from struct import unpack
from ctypes import *
import time
import binascii
import sys

def checksum8bitHonda(data):
	return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF

def checksum8bit(data):
	return 0xff - ((sum(bytearray(data))-1) >> 8)

def validate_checksum(byts, fix=False):
	cksum = len(byts)-8
	fcksum = byts[cksum]
	ccksum = checksum8bitHonda(byts[:cksum]+byts[(cksum+1):])
	fixed = False
	if fix:
		if fcksum != ccksum:
			fixed = True
			byts[cksum] = ccksum
	return byts, fcksum, ccksum, fixed


def format_message(mtype, data):
	ml = len(mtype)
	dl = len(data)
	msgsize = 0x02 + ml + dl
	msg = mtype + [msgsize] + data
	msg += [checksum8bitHonda(msg)]
	assert(msg[ml] == len(msg))
	return msg, ml, dl

class HondaECU(object):

	def __init__(self, device_id=None):
		super(HondaECU, self).__init__()
		self.device_id = device_id
		self.dev = None
		self.error = 0
		self.resets = 0
		self.reset()

	def reset(self):
		if self.dev != None:
			del self.dev
			self.dev = None
		self.dev = Device(self.device_id)

	def setup(self):
		self.dev.ftdi_fn.ftdi_usb_reset()
		self.dev.ftdi_fn.ftdi_usb_purge_buffers()
		self.dev.ftdi_fn.ftdi_set_line_property(8, 1, 0)
		self.dev.baudrate = 10400

	def _break(self, ms, debug=False):
		self.dev.ftdi_fn.ftdi_set_bitmode(1, 0x01)
		self.dev._write(b'\x00')
		time.sleep(ms)
		self.dev._write(b'\x01')
		self.dev.ftdi_fn.ftdi_set_bitmode(0, 0x00)
		self.dev.flush()

	def init(self, debug=False):
		ret = False
		self._break(.070)
		time.sleep(.130)
		self.dev.flush()
		info = self.send_command([0xfe],[0x72], debug=debug, retries=0) # 0xfe <- KWP2000 fast init all nodes ?
		if info != None and ord(info[0]) > 0:
			if ord(info[2]) == 0x72:
				ret = True
		return ret

	def kline(self):
		b = create_string_buffer(2)
		self.dev.ftdi_fn.ftdi_poll_modem_status(b)
		return b.raw[1] & 16 == 0

	def send(self, buf, ml, timeout=.5):
		self.dev.flush()
		msg = "".join([chr(b) for b in buf])
		self.dev._write(msg)
		r = len(msg)
		to = time.time()
		while r > 0:
			r -= len(self.dev._read(r))
			if time.time() - to > timeout: return None
		buf = bytearray()
		r = ml+1
		while r > 0:
			tmp = self.dev._read(r)
			r -= len(tmp)
			buf.extend(tmp)
			if time.time() - to > timeout: return None
		r = buf[-1]-ml-1
		while r > 0:
			tmp = self.dev._read(r)
			r -= len(tmp)
			buf.extend(tmp)
			if time.time() - to > timeout: return None
		return buf

	def send_command(self, mtype, data=[], retries=10, debug=False):
		msg, ml, dl = format_message(mtype, data)
		first = True
		while first or retries > 0:
			first = False
			if debug:
				sys.stderr.write("> [%s]" % ", ".join(["%02x" % m for m in msg]))
			resp = self.send(msg, ml)
			ret = None
			if resp == None:
				if debug:
					sys.stderr.write(" !%d \n" % (retries))
				retries -= 1
				time.sleep(0)
				continue
			else:
				if debug:
					sys.stderr.write("\n")
			if debug:
				sys.stderr.write("< [%s]" % ", ".join(["%02x" % r for r in resp]))
			invalid = (resp[-1] != checksum8bitHonda([r for r in resp[:-1]]))
			if invalid:
				if debug:
					sys.stderr.write(" !%d \n" % (retries))
				retries -= 1
				time.sleep(0)
				continue
			else:
				if debug:
					sys.stderr.write("\n")
			sys.stderr.flush()
			rmtype = resp[:ml]
			rml = resp[ml:(ml+1)]
			rdl = ord(rml) - 2 - len(rmtype)
			rdata = resp[(ml+1):-1]
			return (rmtype, rml, rdata, rdl)

	def do_init_recover(self, debug=False):
		self.send_command([0x7b], [0x00, 0x01, 0x03], debug=debug)
		self.send_command([0x7b], [0x00, 0x01, 0x01], debug=debug)
		self.send_command([0x7b], [0x00, 0x01, 0x02], debug=debug)
		self.send_command([0x7b], [0x00, 0x01, 0x03], debug=debug)
		self.send_command([0x7b], [0x00, 0x02, 0x76, 0x03, 0x17], debug=debug) # seed/key?
		self.send_command([0x7b], [0x00, 0x03, 0x75, 0x05, 0x13], debug=debug) # seed/key?

	def do_init_write(self, debug=False):
		# is this the command to erase the ECU?
		self.send_command([0x7d], [0x01, 0x01, 0x00], debug=debug)
		self.send_command([0x7d], [0x01, 0x01, 0x01], debug=debug)
		self.send_command([0x7d], [0x01, 0x01, 0x02], debug=debug)
		self.send_command([0x7d], [0x01, 0x01, 0x03], debug=debug)
		self.send_command([0x7d], [0x01, 0x02, 0x50, 0x47, 0x4d], debug=debug) # seed/key?
		self.send_command([0x7d], [0x01, 0x03, 0x2d, 0x46, 0x49], debug=debug) # seed/key?

	def do_pre_write(self, debug=False):
		self.send_command([0x7e], [0x01, 0x01, 0x00], debug=debug)
		time.sleep(11)
		self.send_command([0x7e], [0x01, 0x02], debug=debug)
		self.send_command([0x7e], [0x01, 0x03, 0x00, 0x00], debug=debug)
		self.send_command([0x7e], [0x01, 0x01, 0x00], debug=debug)
		self.send_command([0x7e], [0x01, 0x0b, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff], debug=debug) # password?
		self.send_command([0x7e], [0x01, 0x01, 0x00], debug=debug)
		self.send_command([0x7e], [0x01, 0x0e, 0x01, 0x90], debug=debug)
		self.send_command([0x7e], [0x01, 0x01, 0x01], debug=debug)
		self.send_command([0x7e], [0x01, 0x04, 0xff], debug=debug)
		self.send_command([0x7e], [0x01, 0x01, 0x00], debug=debug)

	def do_pre_write_wait(self, debug=False):
		while True:
			info = self.send_command([0x7e], [0x01, 0x05], debug=debug)
			if info[2][1] == 0x00:
				break
		self.send_command([0x7e], [0x01, 0x01, 0x00], debug=debug)
