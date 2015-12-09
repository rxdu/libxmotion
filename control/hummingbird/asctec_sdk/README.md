# MCU

## 1. Set Up Development Environment for HL MCU with AscTec SDK

Install dependent packages:

```
sudo apt-get install openocd
sudo apt-get install libncurses5-dev:i386
sudo apt-get install libx11-dev:i386
sudo apt-get install zlib1g:i386
```

Download GCC Toolchain from [page](http://wiki.asctec.de/display/AR/SDK+Downloads). You can use the included script to set it up or you can put it anywhere you prefer and add the path to your system $PATH. Use this command to check if the tools are set up correctly:

```
$ arm-elf-gcc --version
```

You should get the following message:

```
arm-elf-gcc (GCC) 4.1.1
Copyright (C) 2006 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

Import project into Eclipse from folder /mcu in the repository.

## 2. Update makefile of the Project When Adding New Source Files

**C source code file (*.c)**

Locate to ~Line71 (search "SRCARM" to help locate this place):

Add a new line to inform the compiler a new source file to be compiled

```
SRCARM += folder-name/src/new-file-name.c
```

**Header file (*.h)**

You only need to modify makefile when you create a new folder for header files. If you only add header files to existing /inc folders, then you don't need to do the following step.

Locate to ~Line149 (search "EXTRAINCDIRS")

```
EXTRAINCDIRS += folder-name/inc
```
