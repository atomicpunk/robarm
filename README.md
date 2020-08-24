# robarm
Robotic Arm Controller

  robarm.py: python command line controller for lewansoul robot arms
    Version: 1.0
     Author: Todd Brandt <todd.eric.brandt@gmail.com>
    License: GPV v2

[SETUP]

Add yourself to the dialout group:
$> sudo adduser myusername dialout
then exit and reconnect

Create this file:
$> sudo pico /etc/udev/rules.d/80-usb-xarm.rules

Add this line, it tells udev anyone in group dialout can use the robotarm:
SUBSYSTEM=="hidraw", ATTRS{product}=="LOBOT", GROUP="dialout", MODE="0660"

Then restart udev to trigger the new rule:
$> sudo udevadm control --reload-rules
$> sudo udevadm trigger

Now the robarm.py tool can be used without sudo
