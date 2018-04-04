from __future__ import division, print_function
from pylibftdi import Device
from struct import unpack
from tabulate import tabulate
import argparse

from HondaECU import *

if __name__ == '__main__':
	
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('--debug', action='store_true', help="turn on debugging output")
	pg_temp = parser.add_argument_group('temperature options')
	pg_temp.add_argument('--temp-offset', type=int, default=-40, help="Offset")
	pg_temp.add_argument('--temp-factor-f', type=float, default=1.0, help="Fahrenheit factor")
	pg_temp.add_argument('--temp-factor-c', type=float, default=1.0, help="Celcius factor")
	args = parser.parse_args()
	print(args)
	
	tables = {
		0x71: [0x00, 0x11, 0x20, 0x61, 0x70, 0xd0, 0xd1],
		#0x73: range(256),
		#0x74: range(256)
	}

	# Initialize communication with ECU
	ecu = HondaECU()

	print("==================================")
	print("Initializing ECU communications")
	ecu.init()
	print("==================================")
	print("")

	"""
	Scan tables
	"""
	pdata = {}
	#for j in range(0, 0x4c)[::-1]:
	for j in tables.keys():
		pdata[j] = {}
		#for i in range(0,256):
		for i in tables[j]:
			pdata[j][i] = {}
			print("==================================")
			print("~",i,j,"~")
			if j == 0x72:
				info = ecu.send_command([0x72], [j, i, 0x00, 0x20], debug=args.debug)
			else:
				info = ecu.send_command([0x72], [j, i], debug=args.debug)
			if info:
				a = ord(info[2][0])
				b = ord(info[2][1])
				if info and info[2] > 0:
					if a == 0x71 and (b == 0x11 or b == 0x61):
						data = unpack(">H12B3H", info[2][2:])
						pdata[j][b] = [
							("RPM", data[0]),
							("TPS_volt", data[1]*5/256),
							("TPS_%", data[2]/1.6),
							("ECT_volt", data[3]*5/256),
							("ECT_deg_C", (data[4]*args.temp_factor_c)+args.temp_offset),
							("ECT_deg_F", (data[4]*args.temp_factor_f)+args.temp_offset),
							("IAT_volt", data[5]*5/256),
							("IAT_deg_C", (data[6]*args.temp_factor_c)+args.temp_offset),
							("IAT_deg_F", (data[6]*args.temp_factor_f)+args.temp_offset),
							("MAP_volt", data[7]*5/256),
							("MAP_kpa", data[8]),
							("?UNK1", data[9]),
							("?UNK2", data[10]),
							("BATT_volt", data[11]/10),
							("SPEED_kph", data[12]),
							("IGN_ang", data[13]/10),
							("*INJ_ms", data[14]),
							("?UNK3", data[15])
						]
					elif a == 0x71 and (b == 0xd0):
						data = unpack(">14B", info[2][2:])
						pdata[j][b] = [
							("STARTED", data[1])
						]
					else:
						data = unpack(">%dB" % len(info[2][2:]), info[2][2:])
				print(tabulate(pdata[j][b]))
