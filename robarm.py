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
# sudo pip3 install easyhid pynput
#

import time
import easyhid
import re
import sys
import struct
import termios

def local_echo(enable):
	iflag, oflag, cflag, lflag, ispeed, ospeed, cc = \
		termios.tcgetattr(sys.stdin)
	if enable:
		lflag |= termios.ECHO
	else:
		lflag &= ~termios.ECHO
	new_attr = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
	termios.tcsetattr(sys.stdin, termios.TCSANOW, new_attr)

class XArm():
	dev = None
	verbose = False
	sn = ['claw', 'wristroll', 'wristpitch', 'elbow', 'shoulder', 'base']
	servoinfo = [
		{'id': 1, 'name': sn[0], 'min': 1310, 'mid': 1500, 'max': 2500, 'pos': -1},
		{'id': 2, 'name': sn[1], 'min': 400,  'mid': 1430, 'max': 2600, 'pos': -1},
		{'id': 3, 'name': sn[2], 'min': 500,  'mid': 1500, 'max': 2500, 'pos': -1},
		{'id': 4, 'name': sn[3], 'min': 400,  'mid': 1670, 'max': 2600, 'pos': -1},
		{'id': 5, 'name': sn[4], 'min': 400,  'mid': 1480, 'max': 2600, 'pos': -1},
		{'id': 6, 'name': sn[5], 'min': 400,  'mid': 1570, 'max': 2600, 'pos': -1},
	]
	def __init__(self, verbose=False):
		self.verbose = verbose
		en = easyhid.Enumeration()
		devices = en.find(vid=0x0483, pid=0x5750)

		if self.verbose:
			for dev in devices:
				print(dev.description())

		if len(devices) < 1:
			print('ERROR: No robotic arms found, check for terminators')
			sys.exit(1)

		self.dev = devices[0]
		self.dev.open()
		if self.verbose:
			print('Connected to xArm device')

	def __del__(self):
		if self.verbose:
			print('Closing xArm device')
		if self.dev:
			self.dev.close()
		local_echo(True)

	def itos(self, v):
		lsb = v & 0xFF
		msb = v >> 8
		return lsb, msb

	def servoIndex(self, id):
		s = -1
		if isinstance(id, str):
			if re.match('^[0-9]*$', id):
				s = int(id) - 1
			elif id in self.sn:
				s = self.sn.index(id)
		elif isinstance(id, int):
			s = id - 1
		if s < 0 or s >= len(self.sn):
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

	def moveTo(self, id, pos, time=0):
		s = self.servoInfo(id)
		if isinstance(pos, str):
			if pos in ['min', 'mid', 'max']:
				pos = s[pos]
			elif re.match('^[0-9]*$', pos):
				pos = int(pos)
			else:
				print('ERROR: %s is not a valid position' % pos)
				sys.exit(1)
		s['pos'] = self.clipPos(s, pos)
		t_lsb, t_msb = self.itos(time)
		p_lsb, p_msb = self.itos(s['pos'])
		self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, s['id'], p_lsb, p_msb])

	def moveRel(self, id, dpos, time=0):
		s = self.servoInfo(id)
		if s['pos'] < 0:
			return
		s['pos'] = self.clipPos(s, s['pos'] + dpos)
		t_lsb, t_msb = self.itos(time)
		p_lsb, p_msb = self.itos(s['pos'])
		self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, s['id'], p_lsb, p_msb])

	def move_all(self, poss, time=0):
		for i in range(6):
			self.moveTo(id=i+1, pos=poss[i], time=time)

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


class KeyControl():
	arm = None
	servosel = 6
	def __init__(self, armobj):
		self.arm = armobj

	def run(self):
		self.arm.rest()
		local_echo(False)
		print('Robot Arm Ready')
		with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
			listener.join()
		print('Robot Arm Off')

	def iskey(self, key, chars):
		try:
			if key.char in chars:
				return True
		except:
			pass
		return False

	def on_press(self, key):
		if key == Key.esc or self.iskey(key, ['q', 'x']):
			return False
		if self.iskey(key, ['1', '2', '3', '4', '5', '6']):
			self.servosel = int(key.char)
			return True
		if self.servosel > 0:
			if key == Key.left or key == Key.down:
				self.arm.moveRel(self.servosel, -50, 100)
			elif key == Key.right or key == Key.up:
				self.arm.moveRel(self.servosel, 50, 100)
			elif key == Key.space:
				self.arm.moveTo(self.servosel, 'mid', 500)
		return True

	def on_release(self, key):
		return True

if __name__ == '__main__':
	import argparse

	parser = argparse.ArgumentParser()
	parser.add_argument('-v', '--verbose', action='store_true',
		help='print verbose data')
	parser.add_argument('-set', nargs=3, metavar=('servo', 'value', 'time'),
		help='set a servo to a given position')
	parser.add_argument('-reset', action='store_true',
		help='reset all servos to starting position')
	parser.add_argument('-read', action='store_true',
		help='try to read the servo positions')
	parser.add_argument('-battery', action='store_true',
		help='read the battery voltage')
	parser.add_argument('-control', action='store_true',
		help='control the robotic arm with keypresses')
	args = parser.parse_args()

	if len(sys.argv) < 2:
		parser.print_help()
		sys.exit(1)

	arm = XArm(args.verbose)
	if args.control:
		from pynput.keyboard import KeyCode, Key, Listener
		keycont = KeyControl(arm)
		keycont.run()

	if args.reset:
		arm.rest()
		print(arm.servoinfo)
	elif args.set:
		if not re.match('^[0-9]*$', args.set[2]):
			print('ERROR: time value must be an integer, not %s' % v)
		arm.moveTo(args.set[0], args.set[1], int(args.set[2]))

	if args.battery:
		print('%d mV' % arm.getBattery())
	elif args.read:
		print(arm.read_pos())
