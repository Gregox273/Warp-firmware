# Warp
Baseline firmware for the Warp hardware platform.


## Prerequisites
You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., `apt-get` on Linux or via [MacPorts](https://www.macports.org) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## Building the Warp firmware
First, make sure the environment variable `ARMGCC_DIR` is set correctly (you can check whether this is set correctly, e.g., via `echo $ARMGCC_DIR`. If this is unfamiliar, see [here](http://homepages.uc.edu/~thomam/Intro_Unix_Text/Env_Vars.html) or [here](https://www2.cs.duke.edu/csl/docs/csh.html)). If your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`. If your shell is `tcsh`:

	setenv ARMGCC_DIR <full path to the directory containing bin/arm-none-eabi-gcc>

Alternatively, if your shell is `bash`

	export ARMGCC_DIR=<full path to the directory containing bin/arm-none-eabi-gcc>

(You can check what your shell is, e.g., via `echo $SHELL`.) Second, edit the jlink command file, `tools/scripts/jlink.commands` to include the correct path.

Third, you should be able to build the Warp firmware by

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. See 	`Warp/src/boot/ksdk1.1.0/README.md` for more.

Fourth, you will need two terminal windows. **You will need to run the following two commands within one/two seconds of each other.** In one shell window, run the firmware downloader. On MacOS, this will be:

	/Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 4000 -CommanderScript ../../tools/scripts/jlink.commands

In the second shell window, launch the JLink RTT client. On MacOS, this will be:

	/Applications/SEGGER/JLink/JLinkRTTClient


## Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c`.
