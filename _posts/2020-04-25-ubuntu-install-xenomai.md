---
layout: post
title: "How to install Xenomai RTOS with Ubuntu"
author: "Nick Rotella"
categories: journal
tags: [rtos,robotics,xenomai]
image: xenomai-logo.jpeg
---

This post contains some transcribed notes from my PhD on installing Xenomai 2 with Ubuntu to our robot control computer. The installation was done on a Dell Optiplex 990 desktop having 16GB of RAM and an Intel Core i7-2600 CPU @ 3.40GHz x 8 cores, as well as two 128GB Samsung SSD's. This machine was responsible for communication via Ethernet (UDP) with processors onboard the robot which each control/report sensor data from individual joints. The robot also had one or more Microstrain IMUs which were used for state estimation and report accelerometer/gyroscope data at a high rate (1000Hz) via USB.

![xenomai.svg](../assets/img/xenomai.svg "Xenomai hardware setup"){: .center-image height="400px" width="600px"}

These instructions are out of date at this point since [Xenomai 3 has been released](https://xenomai.org/documentation/xenomai-3/html/README.INSTALL/), but I figured maybe someone will stumble across this and find it somewhat useful especially for installing on older systems. The notes also walk through installing [RTnet](http://www.rtnet.org/) for real-time Ethernet networking and [usb4rt](https://gitlab.tuebingen.mpg.de/nrotella/usb4rt) for real-time USB 1.1 communication, as well as real-time serial and some other machine setup.

* Table of contents:
{:toc}

## Xenomai 2.6.3 Installation

First, install Ubuntu 12.04 following the usual install instructions with manual partitioning. For the Ubuntu installation, 100GB of the first drive was dedicated to the operating system (mount point /) and the rest was left as swap space.  On the second drive, 75GB was dedicated to user home directories and the rest was left to backup space.

### Downloading the sources/configuring Xenomai

As root download Xenomai (as a Debian package) and the Linux Kernel (I used a vanilla kernel and put all the sources in /usr/src).  For details on debian packages, see [this page](https://gitlab.denx.de/Xenomai/xenomai/-/wikis/Building_Debian_Packages).

```bash
cd /usr/src

# download xenomai
wget -O - http://download.gna.org/xenomai/stable/xenomai-2.6.3.tar.bz2 | tar -jxf -
cd xenomai-2.6.3

# create a new debian changelog entry and build the packages in the parent directory
DEBEMAIL="your@email" DEBFULLNAME="Your Name" debchange -v 2.6.3 Release 2.6.3
debuild -uc -us

# install the resulting packages
cd ..
dpkg -i libxenomai1_2.6.3_amd64.deb libxenomai-dev_2.6.3_amd64.deb xenomai-doc_2.6.3_all.deb xenomai-kernel-source_2.6.3_all.deb xenomai-runtime_2.6.3_amd64.deb
```

The packages contain, respectively:

 * xenomai-runtime - Xenomai runtime utilities
 * libxenomai1 - Shared libraries for Xenomai
 * libxenomai-dev - Headers and static libraries for Xenomai
 * xenomai-doc - Xenomai documentation
 * xenomai-kernel-source - Patches and goodies for building the linux kernel

The list of Xenomai-supported Linux kernels can be found in ```/usr/src/xenomai-kernel-source/ksrc/arch/x86/patches/``` - here, we use kernel version 3.8.13 from [kernel.org](http://www.kernel.org).  Download and unzip this kernel and then patch it with xenomai as shown below.

> Note that for this version we needed to apply a patch to the Xenomai source code to fix a latency issue created by running ```xeno latency``` in parallel with ```watch /proc/xenomai/stat``` - see [this thread](http://www.xenomai.org/pipermail/xenomai/2014-March/030303.html) for more details. Maybe not relevant for other versions, omitted from instructions.

```bash
# first we download the linux kernel
cd /usr/src
wget https://www.kernel.org/pub/linux/kernel/v3.x/linux-3.8.13.tar.bz2
tar -jxf linux-3.8.13.tar.bz2

# create symbolic links
ln -s /usr/src/linux-3.8.13 linux
ln -s /usr/src/xenomai-2.6.3 xenomai

# finally, patch the linux kernel for xenomai (making sure that the adeos patch specified matches the linux kernel version)
/usr/src/xenomai/scripts/prepare-kernel.sh --linux=/usr/src/linux --ipipe=/usr/src/xenomai/ksrc/arch/x86/patches/ipipe-core-3.8.13-x86-4.patch 
```

### Kernel configuration

You can use [my kernel config](../download/config_kernel_3.8.13_xeno_2.6.3) if you want - copy it to ```/usr/src/linux/.config``` and then configure the kernel by doing the following:

```bash
cd /usr/src/linux
 
# copy the old kernel config file (optional)
mv ~/Downloads/config_kernel_3.8.13_xeno_2.6.3 /usr/src/linux/.config
 
# run the kernel config GUI and double-check options below
make menuconfig
```

If you copied my config file then all the options suggested below should already be set - you should double-check them to be sure. In the menuconfig GUI, scrolling to the option and hitting keys *Y*, *N*, or *M* will *Enable*, *Disable*, or *Enable as a kernel module* that option, respectively.  If there is no asterisk in the brackets, the option is already disabled.  Brackets like *[]* mean the option can be enabled with *Y*, while brackets like *<>* mean it can either be enabled with *Y* or enabled to be built as a kernel module with *M*.

> **A Note on Kernel Modules**
>
> When the kernel is compiled, modules (sometimes also called "drivers") are either built as loadable (=m) or built-in (=y) and are marked as such in the kernel config file ```/usr/src/KERNELNAME/.config```. While loadable modules can be unloaded and even blacklisted at boot time by editing ```/etc/modprobe.d/blacklist```, there is no *simple* way to unload a built-in module without first recompiling the kernel to make it loadable.  This means that anything you might wish to load/unload on the fly should be marked with (=m).
>
> While ```modprobe -l``` lists all loadable modules, you can check the file ```/lib/modules/KERNELNAME/modules.builtin``` to find built-in modules.  You can get information about a module, including its location, using ```modinfo MODULENAME```.  You can check if particular modules are loaded with ```lsmod``` and load/unload them with ```insmod```/```rmmod``` or using ```modprobe```. Note that all modules are actually installed in ```/lib/modules/KERNELNAME/``` and modules can be loaded at boot time by adding them to ```/etc/modules```.

The kernel configuration options to check are listed below. You can always edit the ```.config``` directly if you mess up in the menuconfig GUI, and see [this page for more configuration details](http://www.tldp.org/HOWTO/SCSI-2.4-HOWTO/kconfig.html).

 * Choose the correct processor (in *Processor Type -> Processor Family*) - I chose *Core 2 / newer Xeon* for an i7 machine.
 * **CONFIG_CPU_FREQ** - Disable (in *Power management->CPU Freq Scaling*)
 * **CONFIG_CPU_IDLE** - Disable (in *Power management -> CPU idle PM support*)
 * **CONFIG_ACPI_PROCESSOR** - Disable(in *Power management->ACPI->processor*)
 * **CONFIG_INPUT_PCSPKR** - Disable (I had to set this directly in the ```.config``` file after being done with menuconfig because I couldn't find the option)
 * **DO NOT DISABLE MSI** - It is now [obsolete](http://permalink.gmane.org/gmane.linux.real-time.xenomai.users/19782)
 * Under *Real-time Subsystem*, you may want to Enable (mark with *M* to build as a kernel module) the real-time serial driver under Drivers.

These and more Xenomai installation details can be found in the [official Xenomai install guide](http://www.xenomai.org/documentation/xenomai-2.6/html/README.INSTALL/) and [this install configuration page](http://xenomai.org/2014/06/configuring-for-x86-based-dual-kernels/).

### Kernel compilation

Once configuration is done, you can exit the GUI and save the configuration file and compile the kernel as follows:

```bash
# compile the kernel (Concurrency level is the number of cores - do not use make -j)
CONCURRENCY_LEVEL=8 CLEAN_SOURCE=no fakeroot make-kpkg --initrd --append-to-version -ipipe-xenomai-2.6.3 --revision 1.0 kernel_image kernel_headers
```

This may take some time (on the order of **tens of minutes**).  Once compilation completes, in ```/usr/src``` you should have two new debian packages: ```linux-image-3.8.13-ipipe-xenomai-2.6.3_1.0_amd64.deb``` and ```linux-headers-3.8.13-ipipe-xenomai-2.6.3_1.0_amd64.deb``` corresponding the kernel source and headers, respectively.

### Bootloader configuration

To access RTOS-specific terminal commands as a non-root user, add your user to the *xenomai* group by issuing:

```bash
usermod -a -G xenomai YOURUSERNAME
```

and then modify the *grub* config which determines bootloader options at ```/etc/default/grub``` to look as follows:

```bash
# If you change this file, run 'update-grub' afterwards to update
# /boot/grub/grub.cfg.
# For full documentation of the options in this file, see:
#   info -f grub -n 'Simple configuration'
GRUB_DEFAULT="2>0"
GRUB_TIMEOUT=10
GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash xeno_nucleus.xenomai_gid=125"
GRUB_CMDLINE_LINUX=""
# Uncomment to enable BadRAM filtering, modify to suit your needs
# This works with Linux (no patch required) and with any kernel that obtains
# the memory map information from GRUB (GNU Mach, kernel of FreeBSD ...)
#GRUB_BADRAM="0x01234567,0xfefefefe,0x89abcdef,0xefefefef"
# Uncomment to disable graphical terminal (grub-pc only)
#GRUB_TERMINAL=console
# The resolution used on graphical terminal
# note that you can use only modes which your graphic card supports via VBE
# you can see them in real GRUB with the command `vbeinfo'
#GRUB_GFXMODE=640x480
# Uncomment if you don't want GRUB to pass "root=UUID=xxx" parameter to Linux
#GRUB_DISABLE_LINUX_UUID=true
# Uncomment to disable generation of recovery mode menu entries
#GRUB_DISABLE_RECOVERY="true"
# Uncomment to get a beep at grub start
#GRUB_INIT_TUNE="480 440 1"
```

The necessary changes are:

 * In the line GRUB_DEFAULT: "2>0" tells the system to automatically boot into "Previous Versions > First Kernel" (this may not be necessary, depending on where the xenomai-patched kernel ends up in the grub menu).
 * In the line GRUB_CMDLINE_LINUX_DEFAULT: this change is needed to pass the xenomai group id as a boot parameter so that this group can run RT commands.  This is the usual place to pass arguments to the kernel.
 * Optionally, you may need to disable your proprietary graphics driver by adding "nomodeset" after "quiet splash" in the same line as above.  See the section on graphics drivers for more info.

Save ```/etc/default/grub```, then run

```bash
update-grub
```

to update the bootloader settings. The next time you reboot, the machine should boot into the Xenomai kernel instead of the existing linux kernel - if not, you'll need to modify the ```GRUB_DEFAULT="2>0"``` line in the bootloader config.

### Kernel installation

Finally, you must install the kernel with:

```bash
cd /usr/src/
dpkg -i linux-image-3.8.13-ipipe-xenomai-2.6.3_1.0_amd64.deb
```

To test the installation was successful, reboot the machine and select the appropriate kernel with grub. After booting a correctly configured kernel, run ```dmesg | grep Xenomai``` and you should see log entries similar these:

```bash
[    2.023815] I-pipe: head domain Xenomai registered.
[    2.023890] Xenomai: hal/x86_64 started.
[    2.023912] Xenomai: scheduling class idle registered.
[    2.023913] Xenomai: scheduling class rt registered.
[    2.024856] Xenomai: real-time nucleus v2.6.3 (Lies and Truths) loaded.
[    2.024857] Xenomai: debug mode enabled.
[    2.025042] Xenomai: SMI-enabled chipset found, but SMI workaround disabled
[    2.025116] Xenomai: starting native API services.
[    2.025117] Xenomai: starting POSIX services.
[    2.025130] Xenomai: starting RTDM services.
```

If you do not, you may have accidentally booted into the wrong kernel; run ```uname -r``` to check the currently booted kernel.  If you have any issues with the Xenomai installation at this point, consult [the official troubleshooting guide](http://www.xenomai.org/documentation/xenomai-2.6/html/TROUBLESHOOTING/).

Note that if you wish to recompile the kernel in the future - for example, with different config parameters - you can use

```bash
dpkg --list | grep linux-image
```

to print which kernels have been installed. Then, use ```dpkg --purge KERNELNAME``` to remove the old kernel and proceed with the kernel config step for the new kernel. For example, the remove the kernel we're just installed, one would use ```dpkg --purge linux-image-3.8.13-ipipe-xenomai-2.6.3```.

## Installation testing

It's important to verify that the RTOS has consistently small latencies for a periodic task, since this is the reason we choose to use it for robot control. Assuming you've added your user to the Xenomai group as above, run ```xeno latency``` to run a simple built-in latency test. Since graphics settings were seen to be related to latency issues, play a bit with the graphics by opening/closing/moving around windows while the test runs. You should see results like:

```bash
== Sampling period: 100 us
== Test mode: periodic user-mode task
== All results in microseconds
warming up...
RTT|  00:00:01  (periodic user-mode task, 100 us period, priority 99)
RTH|----lat min|----lat avg|----lat max|-overrun|---msw|---lat best|--lat worst
RTD|      1.615|      1.923|      9.846|       0|     0|      1.615|      9.846
RTD|      1.615|      1.923|      9.692|       0|     0|      1.615|      9.846
RTD|      1.538|      1.923|     10.230|       0|     0|      1.538|     10.230
RTD|      1.615|      1.923|     10.384|       0|     0|      1.538|     10.384
RTD|      1.615|      1.923|     11.230|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.923|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.923|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     11.076|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     10.538|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     11.076|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     10.615|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     10.076|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.923|       0|     0|      1.538|     11.230
RTD|      1.538|      1.923|     10.538|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     10.923|       0|     0|      1.538|     11.230
RTD|      1.538|      1.923|     10.153|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.615|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|     10.769|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.153|       0|     0|      1.538|     11.230
RTD|      1.538|      1.923|     10.307|       0|     0|      1.538|     11.230
RTD|      1.615|      1.923|      9.538|       0|     0|      1.538|     11.230
RTT|  00:00:22  (periodic user-mode task, 100 us period, priority 99)
RTH|----lat min|----lat avg|----lat max|-overrun|---msw|---lat best|--lat worst
RTD|      1.615|      1.923|     11.384|       0|     0|      1.538|     11.384
RTD|      1.615|      1.923|     10.076|       0|     0|      1.538|     11.384
RTD|      1.538|      1.923|      9.538|       0|     0|      1.538|     11.384
---|-----------|-----------|-----------|--------|------|-------------------------
RTS|      1.538|      1.923|     11.384|       0|     0|    00:00:25/00:00:25
```

Normal values for the max latency *lat max* are from around **0-10 (microseconds)** but this will vary with hardware. Anything above, say, **40 (microseconds)** is an indicator of something bad happening.  It's especially important to verify that ```xeno latency``` outputs reasonable latencies while doing other RTOS tasks, so we need to run a few more tests.

In Xenomai, the file ```/proc/xenomai/stat``` is updated to show task performance and **kernel mode** - the mode is very important because we want to be sure our tasks remain in real-time mode where they have highest priority. To check this, we run:

```bash
watch -n 0.1 cat /proc/xenomai/stat
```

to print the contents of the file every 0.1 seconds. However, there have been [reported issues](https://xenomai.org/pipermail/xenomai/2014-September/031713.html) of this monitoring causing spikes in latencies, so we simultaneously run ```xeno latency``` and check the test results while doing so.  If you see a spike when you start watching, then you may need to search around for a patch for your kernel.

The second latency test to run is to start your robot simulation/control environment and move the windows around while running ```xeno latency```. If there are no latency spikes then your setup is stable.

### Graphics drivers and Xenomai

#### NVIDIA Graphics cards

There are [known issues with the NVidia 'NICE' graphics drivers](http://www.xenomai.org/pipermail/xenomai/2010-October/021864.html). They generate latency spikes when you start up opengl windows.

#### Intel Onboard Graphics i915

On our Dell machines, we had issues with the Intel onboard graphics chip and proprietary driver (i915) causing latency spikes when running programs using *X-window*.  The graphics chip specs are listed below.

```bash
00:02.0 VGA compatible controller: Intel Corporation 2nd Generation Core Processor Family Integrated Graphics Controller (rev 09) (prog-if 00 [VGA controller])
    Subsystem: Dell Device 047e
    Flags: fast devsel, IRQ 16
    Memory at e0c00000 (64-bit, non-prefetchable) [size=4M]
    Memory at d0000000 (64-bit, prefetchable) [size=256M]
    I/O ports at 6000 [size=64]
    Expansion ROM at <unassigned> [disabled]
    Capabilities: [90] MSI: Enable- Count=1/1 Maskable- 64bit-
    Capabilities: [d0] Power Management version 2
    Capabilities: [a4] PCI Advanced Features
    Kernel modules: i915
```

One solution in such a case is to use an external graphics card; we were able to resolve the issue using an AMD card and associated drivers instead.  However, we also lacked the PCI slot space in these small machines for a graphics card once we installed ethernet and USB 1.1 cards needed for other real-time drivers.  As a result, we ended up removing the external graphics card and using the Xorg VESA drivers with the above Intel motherboard graphics chip.  This was accomplished by adding the parameter ```nomodeset``` to the grub command line arguments (right after "quiet splash") in the file ```/etc/default/grub```.  If you use this workaround, you may also need to add the line ```GRUB_GFXMODE=1280x1024x24``` later in this file in order to get decent resolution on a larger monitor. Graphics issues like this seem common.

#### Issues with SSD installs

If you run into issues such as an infinite black/purple screen preventing you from reaching a login prompt, it may be due to the fact that you're booting Ubuntu from an SSD.  One workaround is to add a delay by adding eg ```sleep 2``` in the file ```/etc/init/lightdm.conf``` above the line ```exec lightdm```.  A better solution is to switch to using **gdm** which seems to work fine with SSDs.  This is apparently a common problem which happens randomly.  If all else fails, drop to a virtual terminal as root (CTRL+ALT+F1) and run ```service lightdm restart.``` To switch to gdm, first install it and then run ```dpkg-reconfigure gdm``` and then ```service lightdm stop``` and finally ```service gdm start```.

## Real-Time Networking using RTnet

Our robot, like many others, uses Ethernet (UDP) for network communication between the control computer and the robot's embedded motor controllers. In order to guarantee that commands sent (and sensor data read back) at 1Khz don't get preempted by other lower-priority tasks, we employ [RTnet](http://www.rtnet.org/), an Open-Source hard real-time network protocol stack for Xenomai and RTAI (real-time Linux extensions).

### Installing RTnet

Download and RT-NET (I used the master branch from sourceforge) as follows:

```bash
# again as root
cd /usr/src
git clone git://git.code.sf.net/p/rtnet/code rtnet
cd rtnet
git checkout 7c8ba10513fe7b63873f753ab22d340bf44119a2 # this was a specific version we needed, may not be necessary in general
```

Next, configure the installation in the same way you configured the linux kernel using ```make menuconfig``` as shown below, optionally copying this pre-configured [rtnet_config](../download/rtnet_config) configuration file to ```/usr/src/rtnet/.config``` to start from.  In the configuration setup you should make sure to do the following:

 * Increase the maximum routing table entries to 64 (or however many you'll need) in *Protocol Stack*
 * Enable real-time ethernet capturing (which builds kernel module ```rtcap.ko```) in *Add-Ons*

Run the following to configure and install rtnet:

```bash
# install the old config file (optional)
mv ~/Downloads/rtnet_config /usr/src/rtnet/.config
 
#configure rtnet as needed and then compile and install
make menuconfig
make && make install
 
# copy the rtnet udev config
cp tools/00-rtnet.rules /etc/udev/rules.d/
```

### Network configuration

**Warning! At this point, you will lose network connectivity temporarily as you manually configure networking to use the real-time drivers. This will be remedied once you edit the following two files and reboot.**

First you'll need to remove the linux automatic network manager so that it doesn't interfere with the manual network configuration we do below.

```bash
apt-get remove network-manager
```

Now, setup the network interfaces and udev rules by editing the file ```/etc/udev/rules.d/70-persistent-net.rules```.

The robot control computer has a 2-port *Intel Corporation PRO/1000 PT Dual Port Server Adapter* ethernet card ethernet card which we'll use for RTnet, and 1 motherboard Ethernet port we'll use for non-real-time networking (internet access). First, we must figure out how to address each of these ports. We can find their MAC addresses using the ```lshw``` command for network hardware:

```bash
lshw -C network
```

to list network hardware which is currently installed. On the robot control computer this outputs (abbreviated):

```bash
  *-network:0 DISABLED   
       description: Ethernet interface
       product: 82571EB Gigabit Ethernet Controller
       vendor: Intel Corporation
       physical id: 0
       bus info: pci@0000:01:00.0
       logical name: eth1
       version: 06
       serial: 00:15:17:dc:5d:9c
       .
       .
       .
  *-network:1 DISABLED
       description: Ethernet interface
       product: 82571EB Gigabit Ethernet Controller
       vendor: Intel Corporation
       physical id: 0.1
       bus info: pci@0000:01:00.1
       logical name: eth2
       version: 06
       serial: 00:15:17:dc:5d:9d
       .
       .
       .
  *-network
       description: Ethernet interface
       product: 82579LM Gigabit Network Connection
       vendor: Intel Corporation
       physical id: 19
       bus info: pci@0000:00:19.0
       logical name: eth0
       version: 04
       serial: 78:2b:cb:9e:38:fa
       .
       .
       .
```
The external ethernet card is clearly the **82571EB Gigabit Ethernet Controller**.  The "serial" lines indicate the MAC addresses ```00:15:17:dc:5d:9c``` and ```00:15:17:dc:5d:9d``` for these interfaces; we will use these for RTNet.  The motherboard interface is ```78:2b:cb:9e:38:fa```, and we will use this for non-RT ethernet.


Note the MAC addresses of the ports you wish to make real-time compatible and reorder/name them in ```/etc/udev/rules.d/70-persistent-net.rules``` as ```eth0```, ```eth1```, and so on starting with the **lowest MAC address**.  Note the MAC address of the port you wish to use for non-RT ethernet and number it ```ethX+1``` (the next highest number available).  After reordering and changing ```eth``` assignments, this file looks like:

```bash
# This file was automatically generated by the /lib/udev/write_net_rules
# program, run by the persistent-net-generator.rules rules file.
#
# You can modify it, as long as you keep each rule on a single
# line, and change only the value of the NAME= key.
 
# PCI device 0x8086:/sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 (e1000e)
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="00:15:17:dc:5d:9c", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="e\
th*", NAME="eth0"
 
# PCI device 0x8086:/sys/devices/pci0000:00/0000:00:01.0/0000:01:00.1 (e1000e)
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="00:15:17:dc:5d:9d", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="e\
th*", NAME="eth1"
 
# PCI device 0x8086:/sys/devices/pci0000:00/0000:00:19.0 (e1000e)
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="78:2b:cb:9e:38:fa", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="e\
th*", NAME="eth2"
```

Now, since we uninstalled the network manager, we need to manually configure the non-real-time interface ```eth2```.  This is done by editing the usual networking file ```/etc/network/interfaces``` to look as shown below:

```bash
auto lo
iface lo inet loopback
auto eth2
iface eth2 inet dhcp
```

Now, reboot the machine and run ```ifconfig``` to monitor interface status.  You should see that ```eth2``` is up and you again have internet connectivity!

### Using RTnet

RTnet must be started using a script we write to load the desired real-time networking modules and set up the real-time routing table. We wrote a shell script called ```load_rtnet``` which is use with ```source /usr/local/rtnet/load_rtnet MODE``` where ```MODE``` can be start, stop or restart.  The full script is given below (abbreviated for clarity):

```bash
#! /bin/sh                                                                                           
RTNETPATH=/usr/local/rtnet
case "$1" in
start)

# First unload non-RT modules (required) and load real-time modules
echo "Starting rtnet from $RTNETPATH"
ifconfig eth2 down
rmmod e1000e
sleep 1
insmod $RTNETPATH/modules/rtnet.ko
sleep 1
insmod $RTNETPATH/modules/rtipv4.ko
insmod $RTNETPATH/modules/rtcfg.ko
insmod $RTNETPATH/modules/rtpacket.ko
insmod $RTNETPATH/modules/rtudp.ko
insmod $RTNETPATH/modules/rt_e1000.ko cards=1,1 # for a quad-port Ethernet card, use cards=1,1,1,1 to enable all its ports in RTnet
sleep 1
insmod /usr/local/rtnet/modules/rtcap.ko

# Set up the real-time interfaces rteth0 and rteth1 for the PCI Ethernet ports
echo "Setting up rtifconfig..."
$RTNETPATH/sbin/rtifconfig rteth0 up 192.168.1.100 netmask 255.255.255.0 hw ether 00:15:17:dc:5d:9c
$RTNETPATH/sbin/rtifconfig rteth1 up 192.168.4.100 netmask 255.255.255.0 hw ether 00:15:17:dc:5d:9d
$RTNETPATH/sbin/rtifconfig
 
# Set up routes from port rteth0 to other networked hardware using their IP and MAC addresses
echo "Setting up rtroute..."
$RTNETPATH/sbin/rtroute add 192.168.1.22 0:0:0:12:34:6B dev rteth0
$RTNETPATH/sbin/rtroute add 192.168.1.23 0:0:0:12:34:6C dev rteth0
.
.
.
 
# Set up routes from port rteth1 to other networked hardware using their IP and MAC addresses
$RTNETPATH/sbin/rtroute add 192.168.3.8 0:0:0:32:34:5D dev rteth1
$RTNETPATH/sbin/rtroute add 192.168.3.9 0:0:0:32:34:5E dev rteth1
.
.
.

# Bring up the real-time interfaces we just set up
ifconfig rteth0 up 192.168.1.100 netmask 255.255.255.0 hw ether 00:15:17:dc:5d:9c
ifconfig rteth1 up 192.168.4.100 netmask 255.255.255.0 hw ether 00:15:17:dc:5d:9d
ifconfig

# Re-load non-RT modules
echo "Loading non-rt ethernet..."
modprobe e1000e
sleep 1
ifconfig eth2 up
;;
restart)
$RTNETPATH/load_rtnet stop
$RTNETPATH/load_rtnet start
;;
stop)
echo "Stopping rtnet from $RTNETPATH"
echo "Shutting down rtifconfig..."
$RTNETPATH/sbin/rtifconfig rteth0 down
$RTNETPATH/sbin/rtifconfig rteth1 down
rmmod rtcap.ko
rmmod rt_e1000.ko
sleep 1
rmmod rtpacket.ko
rmmod rtcfg.ko
rmmod rtudp.ko
rmmod rtipv4.ko
#rmmod rtloopback.ko                                                                                 
rmmod rtnet.ko                                                                                
;;
*)
echo $"usage: $0 {start|stop|restart}"
exit 3
;;
esac
:
```

The above script basically unloads non-real-time networking modules/interface **eth2**, loads the real-time modules, sets up the real-time interfaces **rteth0** and **rteth1** , sets up routes for these real-time interfaces, and re-loads the non-real-time interface **eth2**.

Run the script and then run ```dmesg``` to confirm that you see output like (abbreviated):

```bash
[  953.206279] *** RTnet 0.9.13 - built on Nov  5 2014 14:49:59 ***
[  953.206282] RTnet: initialising real-time networking
.
.
.
[  954.260555] e1000: 0000:01:00.0: e1000_probe: (PCI Express:2.5Gb/s:Width x4) 00:15:17:dc:5d:9c
[  954.336507] RTnet: registered rteth0
[  954.336508] e1000: rteth0: e1000_probe: Intel(R) PRO/1000 Network Connection
[  954.384213] e1000: 0000:01:00.1: e1000_probe: (PCI Express:2.5Gb/s:Width x4) 00:15:17:dc:5d:9d
[  954.460152] RTnet: registered rteth1
[  954.460153] e1000: rteth1: e1000_probe: Intel(R) PRO/1000 Network Connection
.
.
.
[  961.879152] e1000e: eth2 NIC Link is Up 1000 Mbps Full Duplex, Flow Control: Rx/Tx
[  961.879186] IPv6: ADDRCONF(NETDEV_CHANGE): eth2: link becomes ready
```

**Very importantly**, check that the MAC addresses in the lines **preceding** ```RTnet: registered rteth0``` and ```RTnet: registered rteth1``` match those used in the ```load_rtnet``` script.  If they do not, change them to match, otherwise the association between port name and MAC address will be wrong no messages will actually get sent. This must be checked because RTNet doesn't necessarily choose interfaces in MAC adress order! Likewise, the real-time interfaces may not necessarily be in physical order on the Ethernet card! You can check by plugging a cable into one port and checking ```dmesg``` for a line like ```rt_e1000: rteth0 NIC Link is Up 100 Mbps Full Duplex```.

### Running RTNet at startup (optional)

To run the RTnet loading script at startup, we must copy it to ```/etc/init.d/``` which is where all *startup scripts* reside. To ensure the script gets run, we must create symbolic links to it in each of the ```/etc/rcN.d/``` directories.

> Briefly, *rc* stands for *run control* and *N* is from *0-6+S*, and the numbers refer to different *runlevels* the system can be initialized into. When a linux machine is started, the kernel loads the root filesystem and reads ```/etc/init/rc-sysinit.conf``` (in Ubuntu) to know which runlevel *N* it should boot into. All scripts linked to in the corresponding ```/etc/rcN.d``` directory are then run.
>
> Linux distributions define these differently, but most reserve levels 0 for shutdown and 6 for reboot; the user can generally modify levels 2-5. In Ubuntu, the GUI interface defaults to 2.  The special runlevel *S* is for scripts before the default runlevel, and should only be used if you need to run something *before all other runlevels*.
>
> To run single commands simply during the boot process, add them to the script ```/etc/rc.local``` which gets run at the end of each multiuser runlevel (traditionally 2-5).
>
> See [this page](http://www.tldp.org/HOWTO/HighQuality-Apps-HOWTO/boot.html) for more information on linux runlevels and [this page](https://help.ubuntu.com/community/UpstartHowto) for Ubuntu-specific information.

We set up runlevels for the RTnet loading script by creating symbolic links to it in the ```rcN.d``` directories with following sequence of commands:

```bash
cp /usr/local/rtnet/load_rtnet /etc/init.d
cd /etc/rc0.d
ln -s ../init.d/load_rtnet K20load_rtnet && cd ../rc1.d
ln -s ../init.d/load_rtnet K20load_rtnet && cd ../rc2.d
ln -s ../init.d/load_rtnet S20load_rtnet && cd ../rc3.d
ln -s ../init.d/load_rtnet S20load_rtnet && cd ../rc4.d
ln -s ../init.d/load_rtnet S20load_rtnet && cd ../rc5.d
ln -s ../init.d/load_rtnet S20load_rtnet && cd ../rc6.d
ln -s ../init.d/load_rtnet K20load_rtnet && cd ../rcS.d
ln -s ../init.d/load_rtnet K20load_rtnet
```

In the link names, *K* denotes that the service should be *killed* and *S* denotes that it should be *started*; service is killed in levels 0, 1, 6 and S and started in all other levels. Note that *start* is passed to the script when the link starts with S and *stop* is passed when the link starts with K, hence why we set up the shell script as such. The number that follows is service *priority* from *01-99*, with *01* being highest priority. We choose *20* somewhat arbitrarily since we have no other service to be run.

## Real-time USB using usb4rt

While our robot's motor controllers were networked with Ethernet, the IMUs onboard for state estimation (and potentially other sensors) are USB. We use a special real-time USB driver called [usb4rt](https://gitlab.tuebingen.mpg.de/nrotella/usb4rt) which unfortunately **only works with USB 1.1 ports/compatible devices**. This worked with the older [Microstrain 3DM-GX3-25 IMU](https://www.microstrain.com/inertial/3dm-gx3-25) we used, however for most sensors USB 1.1 is not an option.

### Required hardware

In order to use real-time USB support, you must have a USB 1.1 compatible card ie one which is UHCI-based since usb4rt only provides a UHCI driver at this time.  Any PCI-based USB card made by Intel/VIA will work.  Of course, you need PCI space for both the real-time ethernet card and for the USB card and potentially also an external graphics card (3 PCI slots total).  Real-time USB 2.0 support is not yet available in Xenomai.

At this time, the real-time USB 1.1 drivers have been tested only with the Microstrain 3dm-gx3-25 and 3dm-gx3-45 IMUs; in theory, any other USB 1.1-compatible device can be used as well.  Here we focus on using these IMUs.

#### A few notes on linux USB (you may choose to skip past this)

The USB protocol is a master/slave system controlled by a single host.  This means USB devices cannot directly talk to one another which alleviates issues of collision avoidance, etc.  USB 1.0 was released in 1996, developed by a number of companies.  USB 1.1 was released in 1998 and allows for a max bandwidth of 12Mbits/s in *Full Speed* mode but in reality is about 8.5Mbits/s under ideal conditions due to overhead (and may be even slower).  There is a *Low Speed* mode limited to 1.5Mbit/s.  USB 2.0 was released in 2000 and featured a *Hi Speed* mode operating at 480Mbit/s (in reality, 280Mbit/s or 35Mbyte/s due to overhead) and a legacy *USB 1.1 Full Speed* mode.  USB 3.0 was released in 2008 and added a *Super Speed* mode for 5Gbit/s (realistically, 4Gbit/s or 500Mbyte/s).

The motherboard provides a USB host controller to act as master using a *Host Controller Driver (HCD)* to interface with the hardware-level *Host Controller Interface (HCI)*.  For USB 1.1, there were two HCIs in use, depending on hardware: Compaq's *Open Host Controller Interface (OHCI)* and Intel's proprietary *Universal Host Controller Interface (UHCI)*.  UHCI requires simpler hardware and thus relies on the controller more heavily, increasing CPU load slightly.  Intel/VIA used UHCI while most other companies used OHCI. For USB 2.0, a single HCD called *Enhanced Host Controller Interface (EHCI)* was developed.  On early setups, EHCI was used for *Hi Speed* mode and either OHCI or UHCI was used for legacy modes.  On modern setups, EHCI implements OHCI/UHCI virtually to indirectly provide legacy support (this is possible because all USB ports are routed through a Rate Matching Hub or RMH). USB 3.0 uses the *Extensible Host Controller Interface (XHCI)* and supports all the above modes.

The HCD you use depends on the HCI your controller hardware uses; use ```lspci -v | grep HCI``` to list controllers and their HCIs and choose your HCD accordingly.  On newer kernels, there exist ```ehci-hcd```, ```uhci-hcd``` and ```ohci-hcd```; you can check if any of these modules are loaded with ```lsmod | grep hcd``` and load/unload them with ```insmod```/```rmmod``` or using ```modprobe```. Note that all modules are actually installed in ```/lib/modules/KERNELNAME``` and modules can be loaded at boot time by adding them to ```/etc/modules```.  You can get information about a module (including its location) using ```modinfo MODULENAME```. 

Physically, a computer has a small number of USB ports; however, the standard supports up to **127 devices**, so *hubs* are used to extend the number of ports.  In theory, self-powered USB devices can pull 500mA per device.  When a USB device is attached, it is assigned a unique device number (1-127) and its device descriptor (which contains device information) is read.

Devices are assigned classes, one of which is Human Interface Device (HID) for peripherals such as a keyboard, mouse, etc.  To get a detailed device list use ```lsusb -v```, and for information about a single device, use eg ```lsusb -D /proc/usb/bus/001/005``` for device 5 on bus 1.  You can easily see assigned device numbers in the kernel log with ```dmesg | grep USB```.

### Installing usb4rt

After installing your USB 1.1-compatible card, you can then install usb4rt with:

```bash
cd /usr/src/
 
# clone the repository
git clone git@gitlab.tuebingen.mpg.de:nrotella/usb4rt.git

# stay on master branch
cd usb4rt
 
# configure the driver (the last option points to the xenomai install directory; it would be /usr/xenomai by default but since we used debian packages this isn't the right place to look)
./configure --enable-drv-cdc-acm --enable-bandw-reclam --with-xeno-user-dir=/usr/xenomai
 
# build and install
make && make install
```

### Configuring usb4rt

Next, we need to unbind the UHCI driver from the 1.1-compatible card and load the real-time drivers in their stead.  Run ```dmesg | grep uhci``` and you should see output which looks like:

```bash
[    2.101228] uhci_hcd: USB Universal Host Controller Interface driver
[    2.101248] uhci_hcd 0000:05:00.0: setting latency timer to 64
[    2.101252] uhci_hcd 0000:05:00.0: UHCI Host Controller
[    2.101255] uhci_hcd 0000:05:00.0: new USB bus registered, assigned bus number 1
[    2.101288] uhci_hcd 0000:05:00.0: irq 16, io base 0x00004020
[    2.101318] usb usb1: Manufacturer: Linux 3.8.13-ipipe-xenomai-2.6.3 uhci_hcd
[    2.101421] uhci_hcd 0000:05:00.1: setting latency timer to 64
[    2.101425] uhci_hcd 0000:05:00.1: UHCI Host Controller
[    2.101428] uhci_hcd 0000:05:00.1: new USB bus registered, assigned bus number 2
[    2.101461] uhci_hcd 0000:05:00.1: irq 17, io base 0x00004000
[    2.101492] usb usb2: Manufacturer: Linux 3.8.13-ipipe-xenomai-2.6.3 uhci_hcd
```

This tells you which USB hubs and PCI addresses your USB 1.1-compatible card is attached to and the IRQ (interrupt) lines it's using with the non-RT *uhci_hcd* driver.  The reason there are two UHCI Host Controllers is because the USB 1.1 ports on the card got split over two hubs/PCI addresses/IRQs here. To figure out which ports correspond to which hub, plug in a device and look for a line in dmesg such as:

```bash
[ 6689.424925] usb 2-1: new full-speed USB device number 9 using uhci_hcd
```

The *usb 2-1* line means that the device was attached to **hub 2 at port 1**.  If you move the device around the USB ports on the card, you'll see this message change - this is a simple way to tell which USB ports get mapped to which hubs.

To confirm this IRQ assignment, run ```cat proc/interrupts``` which should produce output like (truncated):

```bash
           CPU0       CPU1       CPU2       CPU3       CPU4       CPU5       CPU6       CPU7      
 16:        546          0          0         91          0          0          0          0   IO-APIC-fasteoi   uhci_hcd:usb1, ehci_hcd:usb3, nouveau
 17:        932          0      69308          0          0          0          0          0   IO-APIC-fasteoi   uhci_hcd:usb2, ehci_hcd:usb4, snd_hda_intel
```

The first column is the IRQ (interrupt) number, and the last column shows the modules which use that interrupt. Here we see the ```uhci_hcd``` modules, which we need to replace with ```usb4rt```, split across IRQs 16 and 17.

#### Resolving IRQ sharing

The problem in our case is that **we cannot have anything but USB modules on each of these interrupts**, so we need to resolve this IRQ sharing.  If you're very lucky, you'll see nothing other than UHCI controllers on their own IRQ lines; in this case you don't have to worry about IRQ sharing and you can skip this section. Otherwise, follow the [Xenomai guide to IRQ sharing](https://gitlab.denx.de/Xenomai/xenomai/-/wikis/Dealing_With_X86_IRQ_Sharing) and try something like what we did below.

Some machine architectures allow you to assign IRQs in the BIOS, in which case you can assign the USB 1.1 card its own IRQ to prevent conflicts.  Most machines don't allow you to do this, though - you're stuck with the above IRQ assignments. In our case, we have the USB hub3 EHCI and graphics drivers on IRQ 16, and USB hub4 EHCI and sound drivers on IRQ 17.  We now have two choices:

 * Unbind the ehci_hcd driver from hub3 and add "nomodeset" to the kernel boot parameters (in /etc/default/grub after "quiet splash") so that the onboard graphics chip gets used in place of the nouveau graphics driver.  Run update-grub and reboot.
 * Move the mouse, keyboard, etc to ports connected to hub3 and unbind ehci_hcd from hub4 instead.  Also unbind the sound driver, disabling sound output.

In our case we went with the first option because we needed to use the onboard graphics chip and VESA driver anyway (in order to open up a PCI slot for the USB 1.1 card and resolve latency issues). This is just an example of how to resolve tricky IRQ issues.

### Running usb4rt

As for RTnet above, we write a shell script ```load_rtusb``` to set up real-time USB, first by unloading the non-RT drivers and then loading the usb4rt modules.  Once you ```chmod``` it to be executable, usage is ```source load_rtusb MODE``` with MODE being stop/start/restart.

This example is specific to our use-case for the USB IMU. You will need to change the PCI addresses in the ```unbind``` and ```bind``` lines as discussed above, as well as change the ```vendor``` and ```product``` IDs based on ```lsusb -v``` output.

```bash
#! /bin/sh
RTUSBPATH=/usr/local/usb4rt/

case "$1" in
start)

# Unbind the non-RT USB drivers
echo "Starting usb4rt from $RTUSBPATH"
echo -n "0000:05:00.0" > /sys/bus/pci/drivers/uhci_hcd/unbind
echo -n "0000:00:1a.0" > /sys/bus/pci/drivers/ehci-pci/unbind
sleep 1

# Load the usb4rt drivers
insmod /usr/local/usb4rt/modules/rt_usbcore.ko
sleep 1
insmod $RTUSBPATH/modules/rt_uhci_hcd.ko
insmod $RTUSBPATH/modules/rt_cdc_acm.ko vendor=0x199b product=0x3065 start_index=0 # specific to the Microstrain IMU
;;
restart)
$RTUSBPATH/load_rtusb stop
$RTUSBPATH/load_rtusb start
;;
stop)
echo "Stopping usb4rt from $RTUSBPATH"
rmmod rt_cdc_acm
rmmod rt_uhci_hcd
rmmod rt_usbcore
sleep 1
echo -n "0000:05:00.0" > /sys/bus/pci/drivers/uhci_hcd/bind
echo -n "0000:00:1a.0" > /sys/bus/pci/drivers/ehci-pci/bind
;;
*)
echo $"usage: $0 {start|stop|restart}"
exit 3
;;
esac
:
```

You should see similar output as below (truncated) in the system log with ```dmesg``` after running the script:

```bash
[15247.217455] uhci_hcd 0000:05:00.0: remove, state 1
[15247.217462] usb usb1: USB disconnect, device number 1
[15247.217609] uhci_hcd 0000:05:00.0: USB bus 1 deregistered
[15247.217647] ehci-pci 0000:00:1a.0: remove, state 1
[15247.217652] usb usb3: USB disconnect, device number 1
[15247.217653] usb 3-1: USB disconnect, device number 2
[15247.224546] ehci-pci 0000:00:1a.0: USB bus 3 deregistered
[15274.708025] ********** Realtime USB-Core Module 0.0.5 ***********
[15274.708028] RT-USBCORE: Max 16 Controller
.
.
.
[15274.708042] RT-USBCORE: Loading Completed (1152 Byte allocated)
[15275.706935] ********* Realtime Driver for Universal Host Controller 0.0.5 **********
[15275.706939] RT-UHC-Driver: Searching for Universal-Host-Controller
[15275.706943] USB Universal Host Controller found : Vendor = 0x1106, Device = 0x3038, IRQ = 16, IO-Port = 0x00002020 (32 Bytes)
[15275.706944] RT-UHC-Driver: Request IO-Port @ 0x00002020 (32 Byte) for UHC[0] ... [OK]
[15275.706947] RT-UHC-Driver: Request RTDM IRQ 16 ... [OK]
.
.
.
[15276.185863] --- LIST USB-DEVICES -------------------------
[15276.185865]  No HCD VENDOR   PROD  CLS SCLS PROT ->CTRL CTRL-> ->BULK BULK->  ->INT  INT-> ->ISOC ISOC-> STATE SPEED USED
[15276.185867] 001 000 0x199b 0x3065 0x02 0x00 0x00 0x0001 0x0001 0x0002 0x0008 0x0004 0x0000 0x0000 0x0000 00000 00002    X
[15276.185868] --- END USB-DEVICES --------------------------
[15276.185869] rt_cdc_acm[0]: device found, vendor=0x199b, product=0x3065
```

The last line indicating the ```rt_cdc_acm``` driver is loaded is confirmation that it worked.  Similar to RTnet interface ```rteth0```, you should also find the device ```rtser0``` now exists in the folder ```/proc/xenomai/rtdm``` as shown:

```bash
root@perseus:~# cat /proc/xenomai/rtdm/rtser0/information
driver:        rt_cdc_acm
version:    0.5.0
peripheral:    USB CDC ACM
provider:    USB4RT
class:        2
sub-class:    -1
flags:        EXCLUSIVE  NAMED_DEVICE 
lock count:    0
```

#### Double-checking IRQ sharing

If you see a line in ```dmesg``` such as

```bash
[ 602.383262] Xenomai: xnintr_irq_handler: IRQ16 not handled. Disabling IRQ line.
```

at this point, you likely still have an IRQ sharing issue. Again, consult the [Xenomai IRQ sharing guide](https://gitlab.denx.de/Xenomai/xenomai/-/wikis/Dealing_With_X86_IRQ_Sharing). Otherwise, let's confirm there is no IRQ sharing happening.

First run the usual linux ```cat /proc/interrupts``` command and check that the IRQ line previously used for ```uhci_hcd``` has been cleared when those non-real-time USB modules were removed, then check Xenomai's IRQ assignment with ```cat /proc/xenomai/irq``` to confirm that same IRQ (for us, 16) now has ```rt_uhci``` loaded:

```bash
IRQ         CPU0        CPU1        CPU2        CPU3        CPU4        CPU5        CPU6        CPU7
 16:        1000           0           0           0           0           0           0           0         rt_uhci
16640:     1007875     1032112     1434126     1527423      266541      420504      264432      313195         [timer]
16641:           2           1           1           1           1           1           1           3         [reschedule]
16642:           0           1           1           1           1           1           1           1         [timer-ipi]
16643:           0           0           0           0           0           0           0           0         [sync]
16707:           8           0           1           0           0           0           0           0         [virtual]
```

### Running usbrt at startup (optional)

Again, as we did for the RTnet script, we copy the usb4rt script to ```/etc/init.d/``` and make links in ```/etc/rcX.d``` as follows:

```bash
cp /usr/local/usb4rt/load_rtusb /etc/init.d
cd /etc/rc0.d
ln -s ../init.d/load_rtusb K20load_rtusb && cd ../rc1.d
ln -s ../init.d/load_rtusb K20load_rtusb && cd ../rc2.d
ln -s ../init.d/load_rtusb S20load_rtusb && cd ../rc3.d
ln -s ../init.d/load_rtusb S20load_rtusb && cd ../rc4.d
ln -s ../init.d/load_rtusb S20load_rtusb && cd ../rc5.d
ln -s ../init.d/load_rtusb S20load_rtusb && cd ../rc6.d
ln -s ../init.d/load_rtusb K20load_rtusb && cd ../rcS.d
ln -s ../init.d/load_rtusb K20load_rtusb
```

### Running usb4rt with more than one device

Using usb4rt with more than one device is simple: just change the line:

```bash
insmod $RTUSBPATH/modules/rt_cdc_acm.ko vendor=0x199b product=0x3065 start_index=0
```

to, for example:

```bash
insmod $RTUSBPATH/modules/rt_cdc_acm.ko vendor=0x199b,0x199b product=0x3065,0x3065 start_index=0
```

where we have told the driver that two devices will be used.  According to the driver, 16 devices can be added in this way.

### Using usb4rt

Once the driver has been loaded, you can interact with the USB device at the ```rtser0``` interface in much the same way you have interacted with a non-real-time device like ```/dev/ttyACM0``` or ```/dev/ttyUSB0```.

### Debugging usb4rt

If for some reason the above instructions did not work, you can find long output with common debug enabled when drivers have been successfully loaded here.  You may also try the following useful commands to find the problem:

 * Use ```lsusb``` to list USB devices, then choose the one you're interested in and use ```lsusb -vd MANUFACTURER:DEVICE``` to get more details about it
 * You can create your own device rules in ```/etc/udev/```, similar to ```70-persistent-net-rules.d``` (the "70" specifies the order in which these rules are used)

Also, note that older versions of usb4rt on the branch *cdc-acm* required you to limit your system RAM to 4GB.  Failing to do this created communication issues because data got read into memory which the driver didn't know about, causing the read command to never return.  To limit the memory, simply edit ```/etc/default/grub``` and add ```mem=4G``` to the ```GRUB_CMDLINE_LINUX_DEFAULT``` line.  **This should not be necessary for the current version of usb4rt on master branch.**

## Real-time serial communication with RTSerial

[Real-time serial communication in Xenomai](https://wiki.emacinc.com/wiki/Xenomai_RTSerial) is much simpler to set up and use than RTnet or usb4rt. To load the real-time serial driver in Xenomai, issue tthe following commands:

```bash
setserial /dev/ttyS0 uart none
modprobe xeno_16550A io=0x3f8 irq=4 start_index=1
```

To unload the RTSerial driver and revert to the non-RT driver:

```bash
setserial /dev/ttyS0 uart 16550A
```

## Other random notes

Here are some other notes which I wrote down during the above setup which may be useful.

### Solving the "invalid arch independent ELF magic" GRUB issue

If you reach a scary "grub rescue" prompt with the error "invalid arch independent ELF magic," fear not.  This indicates that your grub install got messed up, but your new Ubuntu installation and data are fine.  This may have happened if you chose the wrong USB boot (UEFI instead of Legacy) from the BIOS; if the following short procedure doesn't work, you can try reinstalling and selecting this boot option instead.  Reboot the machine (hard reset if necessary) and insert the install USB again, boot from the USB in the BIOS and now choose "Try Ubuntu."  Note that the grub rescue prompt may prevent you from reaching the BIOS; you probably just didn't hit F12 (or whatever key) at the right time.  I managed to get to the BIOS every time by repeatedly tapping F12 repeatedly as soon as the boot process began.  Once you manage to get to a desktop using the Try Ubuntu option, open a terminal and run the following commands, replacing /dev/sdXY with the mounting point of / chosen in your installation (for example, if / is mounted at /dev/sdb1 then X=b, Y=1):

```bash
sudo mount /dev/sdXY /mnt
sudo grub-install --boot-directory=/mnt /dev/sdX
```

Reboot and everything should work normally again!  If not, you may also have to install the ```grub-efi-amd64``` package from the live CD.  If you absolutely cannot reach the BIOS to get to the live CD, it should also be possible to boot directly from the grub rescue prompt (check online for commands).