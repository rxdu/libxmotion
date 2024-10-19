# HID Device Driver

This module provides driver for the following devices:

- Keyboard
- JoyStick

Two types of drivers are provided:

- Polling driver: Keyboard, Joystick
- Event-driven driver: (KeyboardHandler, JoystickHandler) + HidEventListener

If you don't care about the performance, both drivers can be used. The polling driver is easier to use and can be used
for simple use cases. It creates a thread in the background that reads the device and calls the callback function.
The event-driven driver is more efficient, especially if you need to handle multiple devices. One instance of
HidEventListener can handle multiple devices.

## Find the device

An easy way to find the device is to use the `evtest` tool.

```bash
$ sudo apt-get install evtest
$ sudo evtest
```

You will get a list of devices. Find the one that corresponds to the device you want to use.

```bash
No device specified, trying to scan all of /dev/input/event*
Available devices:
/dev/input/event0:	Sleep Button
/dev/input/event1:	Power Button
/dev/input/event2:	Power Button
/dev/input/event3:	HID Keyboard HID Keyboard
/dev/input/event4:	HID Keyboard HID Keyboard
/dev/input/event6:	MSI WMI hotkeys
/dev/input/event7:	G2Touch Multi-Touch
/dev/input/event8:	USB 2.0 Camera: HD USB Camera
/dev/input/event9:	HDA NVidia HDMI/DP,pcm=3
/dev/input/event10:	Logitech USB Receiver
/dev/input/event11:	Logitech USB Receiver Mouse
/dev/input/event12:	Logitech USB Receiver Consumer Control
/dev/input/event13:	Logitech USB Receiver System Control
/dev/input/event14:	HDA NVidia HDMI/DP,pcm=7
/dev/input/event15:	HDA NVidia HDMI/DP,pcm=8
/dev/input/event16:	HDA NVidia HDMI/DP,pcm=9
/dev/input/event17:	HDA Intel PCH Front Mic
/dev/input/event18:	HDA Intel PCH Rear Mic
/dev/input/event19:	HDA Intel PCH Line
/dev/input/event20:	HDA Intel PCH Headphone Front
/dev/input/event21:	HDA Intel PCH Front Headphone Surround
```

## Device permission

You can check the permission of the device with the `ls` command.

```bash
$ ls -la /dev/input/event8 
crw-rw---- 1 root input 13, 72 Oct 19  2024 /dev/input/event8
```

The device is owned by the `root` user and the `input` group. You can add your user to the `input` group to get access
to the device.

```bash
$ sudo usermod -a -G input $USER
```