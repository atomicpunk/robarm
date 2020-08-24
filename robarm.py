#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Robotic Arm Controller
# Written by Todd Brandt <todd.eric.brandt@gmail.com>
#
# Based on xarm_control.py written by: Maxime Chevalier-Boisvert
# https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
#
# sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
# sudo pip3 install hid easyhid
#

import time
import easyhid
import re
import sys
import struct

class XArm():
	servonames = ['claw', 'wristyaw', 'wristpan', 'elbow', 'shoulder', 'base']
	servoinfo = [
		{'id': 1, 'min': 1300, 'mid': 1500, 'max': 2500, 'name': 'claw'},
		{'id': 2, 'min': 400,  'mid': 1430, 'max': 2600, 'name': 'wristyaw'},
		{'id': 3, 'min': 500,  'mid': 1500, 'max': 2500, 'name': 'wristpan'},
		{'id': 4, 'min': 400,  'mid': 1670, 'max': 2600, 'name': 'elbow'},
		{'id': 5, 'min': 400,  'mid': 1500, 'max': 2600, 'name': 'shoulder'},
		{'id': 6, 'min': 400,  'mid': 1500, 'max': 2600, 'name': 'base'},
	]
	def __init__(self, pid=0x5750):

		# Stores an enumeration of all the connected USB HID devices
		en = easyhid.Enumeration()

		# return a list of devices based on the search parameters
		devices = en.find(vid=0x0483, pid=pid)

		# print a description of the devices found
		for dev in devices:
			print(dev.description())

		assert len(devices) > 0
		self.dev = devices[0]

		# open a device
		self.dev.open()
		print('Connected to xArm device')

	def __del__(self):
		print('Closing xArm device')
		self.dev.close()

	def itos(self, v):
		lsb = v & 0xFF
		msb = v >> 8
		return lsb, msb

	def servoIndex(self, id):
		s = -1
		if isinstance(id, str):
			if re.match('^[0-9]*$', id):
				s = int(id) - 1
			elif id in self.servonames:
				s = self.servonames.index(id)
		elif isinstance(id, int):
			s = id - 1
		if s < 0 or s >= len(self.servonames):
			print('ERROR: %s is not a valid servo' % id)
			sys.exit(1)
		return s

	def servoInfo(self, id):
		return self.servoinfo[self.servoIndex(id)]

	def clipPos(self, info, pos):
		if pos < info['min']:
			return info['min']
		elif pos > info['max']:
			return info['max']
		return pos

	def move_to(self, id, pos, time=0):
		s = self.servoInfo(id)
		pos = self.clipPos(s, pos)

		t_lsb, t_msb = self.itos(time)
		p_lsb, p_msb = self.itos(pos)
		self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, s['id'], p_lsb, p_msb])

	def move_all(self, poss, time=0):
		for i in range(6):
			self.move_to(id=i+1, pos=poss[i], time=time)

	def servos_off(self):
		self.dev.write([0x55, 0x55, 9, 20, 6, 1, 2, 3, 4, 5, 6])

	def read_pos(self):
		self.dev.write([
			0x55, 0x55,
			9,  # Len
			21, # Cmd
			6,  # Count
			1,
			2,
			3,
			4,
			5,
			6
		])

		ret = self.dev.read()
		count = ret[4]
		assert count == 6

		poss = []
		for i in range(6):
			id = ret[5 + 3*i]
			p_lsb = ret[5 + 3*i + 1]
			p_msb = ret[5 + 3*i + 2]
			pos = (p_msb << 8) + p_lsb
			poss.append(pos)

		return poss

	def getBattery(self):
		self.dev.write([0x55, 0x55, 2, 15])
		ret = self.dev.read()
		try:
			out = struct.unpack('H', ret[4:6])
		except:
			return -1
		return out[0]

	def rest(self):
		self.move_all([
			self.servoinfo[0]['mid'],
			self.servoinfo[1]['mid'],
			self.servoinfo[2]['mid'],
			self.servoinfo[3]['mid'],
			self.servoinfo[4]['mid'],
			self.servoinfo[5]['mid']
		], time=1500)
		time.sleep(2)
		self.servos_off()

if __name__ == '__main__':
	import argparse

	parser = argparse.ArgumentParser()
	parser.add_argument('-set', nargs=3, metavar=('servo', 'value', 'time'),
		help='set a servo to a given position')
	parser.add_argument('-reset', action='store_true',
		help='reset all servos to starting position')
	parser.add_argument('-read', action='store_true',
		help='try to read the servo positions')
	parser.add_argument('-battery', action='store_true',
		help='read the battery voltage')
	args = parser.parse_args()

	if len(sys.argv) < 2:
		parser.print_help()
		sys.exit(1)

	arm = XArm()
	if args.reset:
		arm.rest()
	elif args.battery:
		print('%d mV' % arm.getBattery())
	elif args.read:
		print(arm.read_pos())
	elif args.set:
		for v in [args.set[1], args.set[2]]:
			if not re.match('^[0-9]*$', v):
				print('ERROR: value must be an integer, not %s' % v)
		arm.move_to(args.set[0], int(args.set[1]), int(args.set[2]))
