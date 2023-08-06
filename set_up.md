# Franka: Getting Started

[TOC]

My operating system: Ubuntu 20.04. Kernel: 5.15.0-46-generic

**NOTE**: in some code fences, I add `$` before the command. First, it can avoid you to copy and paste simply, since it may contain some numbers which you need to change according to your situation. Second, the command starting with a `$` can be differentiated from the output of the terminal.

## Install the real-time kernel

Although in Franka on-line documentation, the real-time kernel can be set up after install `libfranka` and `franka_ros`, but I did this step first, for it is the most annoying step.

First, install the necessary dependencies:

```bash
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
```

Then, you have to decide which kernel version to use. To find the one you are using currently, use `uname -r`. Real-time patches are only available for select kernel versions. We recommend choosing the version closest to the one you currently use. If you choose a different version, simply substitute the numbers.

You have to download 4 files from [the Internet](https://www.kernel.org/pub/linux/kernel), `.xz` and `.sign` file of the kernel and `.xz` and `.sign` file of the patch:

- https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.44.tar.xz
- https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.44.tar.sign
- https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/patch-5.15.44-rt46.patch.xz
- https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/patch-5.15.44-rt46.patch.sign

And decompress them with:

```bash
$ xz -d linux-5.15.44.tar.xz
$ xz -d patch-5.15.44-rt46.patch.xz
```

`-d` means deleting the original file. So, there are still 4 files.

### [Verifying file integrity](https://frankaemika.github.io/docs/installation_linux.html#verifying-file-integrity)

The `.sign` files can be used to verify that the downloaded files were not corrupted or tampered with. You can use `gpg2` to verify the `.tar` archives:

```bash
gpg2 --verify linux-*.tar.sign
gpg2 --verify patch-*.patch.sign
```

If your output is similar to the following:

```bash
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID <the whole id>
gpg: Can't check signature: No public key
```

You have to first download the public key of the person who signed the above file. As you can see from the above output, it has the ID (maybe you do not need `0x` before `<the whole id>`):

```bash
$ gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 0x<the whole id>
gpg: key xxxxxxxxxxxxxxxx: 1 duplicate signature removed
gpg: key xxxxxxxxxxxxxxxx: public key "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" imported
gpg: Total number processed: 1
gpg:               imported: 1
```

Having downloaded the keys, you can now verify the sources. Here is an example of a correct output (you can see `Good signature`):

```bash
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E
```

### Compiling the kernel

Once you are sure the files were downloaded properly, you can extract the source code and apply the patch:

```bash
$ tar xf linux-5.14.44.tar
$ cd linux-5.15.44/
$ patch -p1 < ../patch-5.15.44-rt46.patch
```

Next copy your currently booted kernel configuration as the default config for the new real time kernel:

```bash
$ cp -v /boot/config-$(uname -r) .config
'/boot/config-4.15.0-30-generic' -> '.config'
```

Now you can use this config as the default to configure the build:

```bash
make menuconfig
```

The command brings up a terminal interface in which you can configure the preemption model. Navigate with the arrow keys to *General Setup* > *Preemption Model* and select *Fully Preemptible Kernel (Real-Time)*.

After that navigate to *Cryptographic API* > *Certificates for signature checking* (at the very bottom of the list) > *Provide system-wide ring of trusted keys* > *Additional X.509 keys for default system keyring* (this step is from the [documentation](https://frankaemika.github.io/docs/installation_linux.html#compiling-the-kernel), but I did not do that).

Possible error:

```bash
$ make menuconfig
  HOSTCC  scripts/kconfig/lxdialog/util.o
  HOSTCC  scripts/kconfig/lxdialog/yesno.o
  HOSTCC  scripts/kconfig/confdata.o
  HOSTCC  scripts/kconfig/expr.o
  LEX     scripts/kconfig/lexer.lex.c
  YACC    scripts/kconfig/parser.tab.[ch]
  HOSTCC  scripts/kconfig/lexer.lex.o
  HOSTCC  scripts/kconfig/menu.o
  HOSTCC  scripts/kconfig/parser.tab.o
  HOSTCC  scripts/kconfig/preprocess.o
  HOSTCC  scripts/kconfig/symbol.o
  HOSTCC  scripts/kconfig/util.o
  HOSTLD  scripts/kconfig/mconf
.config:8758:warning: symbol value 'm' invalid for ASHMEM
.config:9810:warning: symbol value 'm' invalid for ANDROID_BINDER_IPC
.config:9811:warning: symbol value 'm' invalid for ANDROID_BINDERFS
Your display is too small to run Menuconfig!
It must be at least 19 lines by 80 columns.
make[1]: *** [scripts/kconfig/Makefile:48: menuconfig] Error 1
make: *** [Makefile:624: menuconfig] Error 2
```

*Solution*: just maximize the window size of the terminal.

You may modify several lines in `.config` manually.

```bash
$ scripts/config --set-str SYSTEM_TRUSTED_KEYS ""
$ cat .config | grep SYSTEM_TRUSTED_KEY # for seeing the changes in .config
CONFIG_SYSTEM_TRUSTED_KEYRING=y
CONFIG_SYSTEM_TRUSTED_KEYS=""
```

```bash
$ scripts/config --disable SYSTEM_REVOCATION_KEYS
$ cat .config | grep SYSTEM_REVOCATION # for seeing the changes in .config
CONFIG_SYSTEM_REVOCATION_LIST=y
CONFIG_SYSTEM_REVOCATION_KEYS=""
```

If you do not need the debug information of compiling, comment `CONFIG_DEBUG_INFO=y` in `.config`:

```bash
$ cat .config | grep DEBUG_INFO
# CONFIG_DEBUG_INFO=y
# CONFIG_DEBUG_INFO_REDUCED is not set
# CONFIG_DEBUG_INFO_COMPRESSED is not set
# CONFIG_DEBUG_INFO_SPLIT is not set
CONFIG_DEBUG_INFO_DWARF_TOOLCHAIN_DEFAULT=y
# CONFIG_DEBUG_INFO_DWARF4 is not set
# CONFIG_DEBUG_INFO_DWARF5 is not set
CONFIG_DEBUG_INFO_BTF=y
CONFIG_DEBUG_INFO_BTF_MODULES=y
```

Afterwards, you are ready to compile the kernel:

```bash
make -j$(nproc) deb-pkg
```

As this is a lengthy process, set the multithreading option `-j` to the number of your CPU cores. However, if you set `-j` larger than 1, you cannot get the detail when you occur an error. Finally, you are ready to install the newly created package. The exact names depend on your environment, but you are looking for `headers` and `images` packages without the `dbg` suffix. To install:

```bash
sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb
```

Possible errors:

1. ```bash
   make[2]: *** [debian/rules:7: build-arch] Error 2
   dpkg-buildpackage: error: debian/rules binary subprocess returned exit status 2
   make[1]: *** [scripts/Makefile.package:77: deb-pkg] Error 2
   make: *** [Makefile:1533: deb-pkg] Error 2
   ```

   *Solution*: `scripts/config --set-str SYSTEM_TRUSTED_KEYS ""`;

2. ```bash
   make[1]: *** No rule to make target 'debian/canonical-revoked-certs.pem', needed by 'certs/x509_revocation_list'.  Stop.
   ```

   *Solution*: `scripts/config --disable SYSTEM_REVOCATION_KEYS`;

3. ```bash
   /bin/sh: 1: zstd: not found
   ```

   *solution*: `sudo apt install zstd`;

4. ```bash
   BTF: .tmp_vmlinux.btf: pahole (pahole) is not available
   Failed to generate BTF for vmlinux
   Try to disable CONFIG_DEBUG_INFO_BTF
   ```

   *solution*: `sudo apt install dwarves`;

5. If you get some error similar to the grammar error of the C++ source code:

   ```bash
   In file included from ./include/Tinux/spinlock.h:54,
   				 from kernel/locking/rtmutex_api.c:5:
   kernel/locking/rtmutex.c: In function '_rt_mutex_slowlock':
   kernel/locking/rtmutex.c:1389:47: error: 'flags' undeclared (first use in this function); did you mean 'fls'?
   1389 | raw_spin_unlock_irqrestore(&lock->wait_lock, flags);
        |                                              ^~~~~
   ./include/linux/typecheck.h:11:9: note: in definition of macro 'typecheck'
     11 | typeof(x)_dummy2；\
        |        ^
   ```

    I have no idea but downloading a new kernel of different version, and try again!

   

### Verifying the new kernel

Restart your system. The Grub boot menu should now allow you to choose your newly installed kernel. To see which one is currently being used, see the output of the `uname -a` command. It should contain the string `PREEMPT RT` and the version number you chose. Additionally, `/sys/kernel/realtime` should exist and contain the the number `1`.

If you encounter errors that you fail to boot the new kernel see [Cannot boot realtime kernel because of “Invalid Signature”](https://frankaemika.github.io/docs/troubleshooting.html#troubleshooting-realtime-kernel)

### Allow a user to set real-time permissions for its processes

After the `PREEMPT_RT` kernel is installed and running, add a group named realtime and add the user controlling your robot to this group:

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Afterwards, add the following limits to the realtime group in `/etc/security/limits.conf`:

```bash
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

The limits will be applied after you log out and in again.

## Install the NVIDIA driver in the real-time kernel

### Reinstalling the driver in the non real-time kernel

First, you need to reinstall the driver in the non real-time kernel. To uninstall original driver, run

```bash
$ sudo apt remove --purge nvidia*
```

A recommended way is using your web browser navigate to the [official Nvidia ](https://www.nvidia.com/Download/index.aspx)website and download an appropriate driver (a `.run` file) for your Nvidia graphic card, so that you can reuse this `.run` file to install the driver of the same version in the real-time kernel. Before installing, you have to disable X server. Press `Ctrl + Alt + F1` to open tty1, which is one of six virtual consoles that Ubuntu provides. Now you can switch back to the GUI by press `Ctrl + Alt + F7` or `Ctrl + Alt + F8`. You need to stop the X service by 

```bash
$ sudo service lightdm stop
```

Now the GUI is not accessible. You can install the NVIDIA driver by

```bash
$ sudo bash NVIDIA-xxx.run
```

**NOTE**: It may ask you something about updating the X configuration, you are suggested to choose *"No"*, otherwise you might not able to reopen the GUI after installing the driver.

**NOTE**: It may ask you something about 32-bit compatibility libraries, choose "*No*".

**NOTE**: It may ask you something about *"dkms"*, choose "*No*", otherwise the installation may fail (but you can try it).

After you are told the installation is completed, run

```bash
$ nvidia-smi
```

to verify  the installation, and run

```bash
$ sudo service lightdm start
```

so that you can switch back to the GUI.

**NOTE**: If you still get stuck in the tty1, the only way is to uninstall the driver you have just installed:

```bash
$ sudo bash NVIDIA-xxx.run --uninstall
```

reboot your computer and install it again.

Hopefully you have managed to install the NVIDIA driver in the non real-time kernel. Now, switch to the real-time kernel. Since NVIDIA binary drivers are not supported on `PREEMPT_RT` kernels, you cannot install the driver just as in the non real-time kernel simply.

### Disabling nouveau

Before you start install the driver in the real-time kernel, you need to disable `nouveau`. Since in my computer `nouveau` has already been disabled in my Ubuntu, I just skip step. You can learn how to disable nouveau from the Internet.

### Installing NVIDIA driver in the real-time kernel

There are two methods to install it. For the first way, you can refer to [this blog](https://blog.csdn.net/tang05505622334/article/details/103477086). Run this script:

```bash
#!/bin/bash

BUILD_BASE=`pwd`
NV_FILE="NVIDIA-Linux-x86_64-515.65.01.run"  # modify this .run file to what you downloaded and installed before
# NV_URL="https://us.download.nvidia.cn/XFree86/Linux-x86_64/430.50/${NV_FILE}"  # If you have downloaded the installation file before, you don't have to download from the Internet again. Otherwise, it will be very slow.
NEED_TO_COMPILE_NV_KO=1

function clean_env() {

    [ -d ./${NV_DIR} ] && rm -rf ./${NV_DIR}
}

function check_env() {
    
    # check if in rt kernel
    uname -r | grep rt 1>/dev/null 2>&1 
    if [ $? -ne 0 ]
    then
        echo "Not in rt kernel, Please install apollo kernel and reboot machine first."
        exit 2
    fi

    # check if nv ko already in kernel
    if [ ! -f /lib/modules/`uname -r`/kernel/drivers/video/nvidia.ko ]
    then
        export NEED_TO_COMPILE_NV_KO=1
    fi
}

function prepare_nv() {

    ## download nv install file from nvidia home page 
    #if [ ! -f ./${NV_FILE} ]
    #then
    #   echo "Downloading ${NV_FILE} from nvidia website..."
    #    wget ${NV_URL} -O ${NV_FILE}
    #    if [ $? -ne 0 ]
    #    then
    #        echo "Downloading ${NV_FILE} failed, please check your network connection!"
    #        rm -rf ./${NV_FILE}
    #        exit 1
    #    fi
    #fi
    ########### You do not need this function if you have already download the .run file ############

    # +x 
    chmod +x ./${NV_FILE}
    echo "Extracting nvidia install run file..."
    ./${NV_FILE} -x 1>/dev/null 2>&1
    NV_DIR="`echo ${NV_FILE} | awk -F '.run' '{print $1}'`"
    NV_VERSION="`echo ${NV_FILE} | awk -F '-' '{print $4}' | awk -F '.run' '{print $1}'`"

    export NV_DIR
    export NV_VERSION
    export NVIDIA_SOURCE="${NV_DIR}/kernel"
}

function install_lib() {
   
    NV_LIB_OUTPUT_PATH="/usr/lib/x86_64-linux-gnu/"
    NV_BIN_OUTPUT_PATH="/usr/bin/"

    [ -f ./${NV_DIR}/libnvidia-ml.so.${NV_VERSION} ] && /bin/cp -f ./${NV_DIR}/libnvidia-ml.so.${NV_VERSION} ${NV_LIB_OUTPUT_PATH}
    [ -f ./${NV_DIR}/libnvidia-fatbinaryloader.so.${NV_VERSION} ] && /bin/cp -f ./${NV_DIR}/libnvidia-fatbinaryloader.so.${NV_VERSION} ${NV_LIB_OUTPUT_PATH}
    [ -f ./${NV_DIR}/libnvidia-ptxjitcompiler.so.${NV_VERSION} ] && /bin/cp -f ./${NV_DIR}/libnvidia-ptxjitcompiler.so.${NV_VERSION} ${NV_LIB_OUTPUT_PATH}
    [ -f ./${NV_DIR}/libcuda.so.${NV_VERSION} ] && /bin/cp -f ./${NV_DIR}/libcuda.so.${NV_VERSION} ${NV_LIB_OUTPUT_PATH}
    [ -f ./${NV_DIR}/nvidia-modprobe ] && /bin/cp -f ./${NV_DIR}/nvidia-modprobe ${NV_BIN_OUTPUT_PATH}
    [ -f ./${NV_DIR}/nvidia-smi ] && /bin/cp -f ./${NV_DIR}/nvidia-smi ${NV_BIN_OUTPUT_PATH}

    chmod +x /usr/bin/nvidia*
    chmod +s /usr/bin/nvidia-modprobe

    # link for nvidia
    /bin/rm -rf /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1  /usr/lib/x86_64-linux-gnu/libnvidia-ml.so
    /bin/ln -s /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.${NV_VERSION} /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1
    /bin/ln -s /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-ml.so

    /bin/rm -rf /usr/lib/x86_64-linux-gnu/libcuda.so  /usr/lib/x86_64-linux-gnu/libcuda.so.1
    /bin/ln -s /usr/lib/x86_64-linux-gnu/libcuda.so.${NV_VERSION} /usr/lib/x86_64-linux-gnu/libcuda.so.1
    /bin/ln -s /usr/lib/x86_64-linux-gnu/libcuda.so.1 /usr/lib/x86_64-linux-gnu/libcuda.so

    # take effect
    /sbin/ldconfig 1>/dev/null 2>&1
}

function build_nv() {

    if [ ${NEED_TO_COMPILE_NV_KO} == 0 ]
    then
        return
    fi

    NVIDIA_MOD_REL_PATH='kernel/drivers/video'
    NVIDIA_OUTPUT_PATH="/lib/modules/`uname -r`/${NVIDIA_MOD_REL_PATH}"
    CPUNUM=`cat /proc/cpuinfo | grep processor | wc | awk -F " " '{print $1}'`

    export IGNORE_PREEMPT_RT_PRESENCE=true
    cd ${NVIDIA_SOURCE} && make -j ${CPUNUM} module
    cd ${BUILD_BASE}

    unset IGNORE_PREEMPT_RT_PRESENCE

    mkdir -p ${NVIDIA_OUTPUT_PATH}

    [ -f ${NVIDIA_SOURCE}/nvidia.ko ] && cp ${NVIDIA_SOURCE}/nvidia.ko ${NVIDIA_OUTPUT_PATH}
    [ -f ${NVIDIA_SOURCE}/nvidia-modeset.ko ] && cp ${NVIDIA_SOURCE}/nvidia-modeset.ko ${NVIDIA_OUTPUT_PATH}
    [ -f ${NVIDIA_SOURCE}/nvidia-drm.ko ] && cp ${NVIDIA_SOURCE}/nvidia-drm.ko ${NVIDIA_OUTPUT_PATH}
    [ -f ${NVIDIA_SOURCE}/nvidia-uvm.ko ] && cp ${NVIDIA_SOURCE}/nvidia-uvm.ko ${NVIDIA_OUTPUT_PATH}

    depmod -a
}

# check environment
check_env

# prepare for nvidia
prepare_nv

# build nvidia.ko
build_nv

# install user lib
install_lib

# clean environment
clean_env

echo "Done to install nvidia kernel driver and user libraries."
```

**NOTE**: remember to modify the 4th line. Hopefully you have succeed to install the NVIDIA driver. Probably, however, you may get the following error:

```bash
FATAL: modpost: GPL-incompatible module xxx.ko uses GPL-only symbol 'xxx'
```

In this case, you can try

```bash
$ sudo IGNORE_PREEMPT_RT_PRESENCE=1 bash NVIDIA-xxx.run
```

Or you can try (you are expected to change `515` to what you need)

```bash
$ sudo IGNORE_PREEMPT_RT_PRESENCE=1 apt install nvidia-driver-515
```

## Setting up ROS Noetic

You can directly follow the [documentation of ROS](http://wiki.ros.org/noetic/Installation/Ubuntu). `franka_ros2` is currently beta software, so installing ROS 1 is okay.

In Yourong Zhang's note, he said that the last two steps

```bash
sudo rosdep init
rosdep update
```

were very slow. I found that, however, the network in the PKU Library was fine. These steps took only a few seconds. If you get stuck in this problem indeed, you can refer to this [blog](https://blog.csdn.net/weixin_44023934/article/details/121242176).

Finally, verify ROS Neotic. Run

```bash
$ roscore &
```

and press `ctrl+C` to exit. Then, run

```bash
$ rqt_graph
```

After unticking the checkbox in the line of *"Hide"*, you can see *"/rosout"*.

Possible errors:

1. ```bash
    $ roscore &
    [1] 69514
    $ ... logging to /home/xxx/.ros/log/xxx.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    
    started roslaunch server http://xxx:43423/
    ros_comm version 1.15.14
    
    
    SUMMARY
    ========
    
    PARAMETERS
     * /rosdistro: noetic
     * /rosversion: 1.15.14
    
    NODES
    
    auto-starting new master
    process[master]: started with pid [69541]
    ROS_MASTER_URI=http://xxx:11311/
    
    setting /run_id to 85211a3e-3d4d-11ed-930e-f31dcf646ee6
    ERROR: cannot launch node of type [rosout/rosout]: Cannot locate node of type [rosout] in package [rosout]. Make sure file exists in package path and permission is set to executable (chmod +x)
    RLException: failed to start core service [/rosout]
    The traceback for the exception was written to the log file
    [master] killing on exit
    ```

    *Solution*: `source /opt/ros/noetic/setup.bash`

2. ```bash
    $ sudo apt install ros-noetic-desktop-full
    Reading package lists... Done
    Building dependency tree       
    Reading state information... Done
    Some packages could not be installed. This may mean that you have
    requested an impossible situation or if you are using the unstable
    distribution that some required packages have not yet been created
    or been moved out of Incoming.
    The following information may help to resolve the situation:
    
    The following packages have unmet dependencies:
     ros-noetic-desktop-full : Depends: ros-noetic-desktop but it is not going to be installed
                               Depends: ros-noetic-perception but it is not going to be installed
                               Depends: ros-noetic-simulators but it is not going to be installed
                               Depends: ros-noetic-urdf-sim-tutorial but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    ```

    This error may occur when you just upgraded Ubuntu 18.04 to 20.04. You can just simply run 

    ```bash
    $ sudo apt install ros-noetic-desktop
    Reading package lists... Done
    Building dependency tree       
    Reading state information... Done
    Some packages could not be installed. This may mean that you have
    requested an impossible situation or if you are using the unstable
    distribution that some required packages have not yet been created
    or been moved out of Incoming.
    The following information may help to resolve the situation:
    
    The following packages have unmet dependencies:
     ros-noetic-desktop : Depends: ros-noetic-viz but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    ```

    Oh! Something similar happened again. You may continue to run:

    ```bash
    $ sudo apt install ros-noetic-viz
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     ros-noetic-viz : Depends: ros-noetic-rqt-common-plugins but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    $ sudo apt install ros-noetic-rqt-common-plugins
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     ros-noetic-rqt-common-plugins : Depends: ros-noetic-rqt-image-view but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    $ sudo apt install ros-noetic-rqt-image-view
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     ros-noetic-rqt-image-view : Depends: ros-noetic-cv-bridge but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    $ sudo apt install ros-noetic-cv-bridge
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     ros-noetic-cv-bridge : Depends: libopencv-dev but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    $ sudo apt install libopencv-dev
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     libopencv-dev : Depends: libopencv-calib3d-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-contrib-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-features2d-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-highgui-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-objdetect-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-stitching-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-superres-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-videoio-dev (= 4.2.0+dfsg-5) but it is not going to be installed
                     Depends: libopencv-videostab-dev (= 4.2.0+dfsg-5) but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    $ sudo apt install libopencv-calib3d-dev
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     libopencv-calib3d-dev : Depends: libopencv-features2d-dev (= 4.2.0+dfsg-5) but it is not going to be installed
    E: Unable to correct problems, you have held broken packages.
    ```

    It seems like the error will never stop! Don't give up! After several `sudo apt install`s, you may reach a leave of the 'error tree' for which you are running *Depth First Search*:

    ```bash
    $ sudo apt install libavutil-dev
    <Repeated output was omitted>
    The following packages have unmet dependencies:
     libavutil-dev : Depends: libavutil56 (= 7:4.2.7-0ubuntu0.1) but 7:4.3.2-0york0~18.04 is to be installed
    E: Unable to correct problems, you have held broken packages.
    ```

    If you try to install `libavutil56`, you will told that it has already been installed. But the output above indicates that it supports Ubuntu 18.04, but not 20.04. So, just uninstall it:

    ```bash
    $ sudo apt remove --purge libavutil56
    ```

    and install it again:

    ```bash
    $ sudo apt install libavutil56
    ```

    Then you can install everything backwards along your searching path to `ros-noetic-desktop-full`. Other branches you skipped during the DFS are probably not necessary to explore again.

3. ```bash
    RLException: roscore cannot run as another roscore/master is already running.
    Please kill other roscore/master processes before relaunching.
    The ROS_MASTER_URI is http://:11311/
    The traceback for the exception was written to the log file
    ```

    *Solution*: you can try `kill -9 <pid>` or `killall -9 <pid>` or `killall -9 rosmaster` repeatedly until you can run `roscore &` successfully. The PID of the process can be accessed by `ps aux|grep roscore`.

## Install `libfranka` and `franka_ros`

Binary packages for `libfranka` and `franka_ros` are available from the ROS repositories:

```bash
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```

**NOTE**: `apt` may only have the latest version of `libfranka` and `franka-ros`, but it may not be compatible with the robot system or the robot version. If so, you have to [build them from source](https://frankaemika.github.io/docs/installation_linux.html#building-from-source).

***IMPORTANT***: remember to install franka-ros >= 0.10.0, otherwise `panda-moveit-config` will fail!

**NOTE**: Do not simply copy-and-paste from the documentation. The documentation refers to some path of a directory `/path/to/desired/folder` or something else. Please remember to replace it with the correct path.

**NOTE**: Any whitespace included in the path may cause errors.

## Operating the robot

pass

## Verifying the connection

###  Visualizing the robot in ros

```bash
roslaunch franka_visualization franka_visualization.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda
```

Possible errors:

1. ```bash
   RLException: [franka_visualization.launch] is neither a launch file in package [franka_visualization] nor is [franka_visualization] a launch file name
   The traceback for the exception was written to the log file
   ```

   *Solution*: `source <path to catkin_ws>/devel/setup.sh` , and run in the same terminal.

3. ```bash
   xacro: error: expected exactly one input file as argument
   RLException: Invalid <param> tag: Cannot load command parameter [robot_description]: command [['/opt/ros/noetic/lib/xacro/xacro', '--inorder', '/home/cdm/Documents/Undergraduate', 'Research/Model_based_robot/catkin_ws/src/franka_ros/franka_description/robots/panda_arm.urdf.xacro', 'hand:=true']] returned with code [2]. 
   
   Param xml is <param name="robot_description" command="$(find xacro)/xacro --inorder $(find franka_description)/robots/panda_arm.urdf.xacro hand:=$(arg load_gripper)"/>
   The traceback for the exception was written to the log file
   ```

   *Reason*:  the path `/home/cdm/Documents/Undergraduate Research/` includes a whitespace.

   *Solution*: delete the whitespace and reinstall `franka-ros`.

4. ```bash
   process[joint_state_publisher-4]: started with pid [23397]
   ERROR: cannot launch node of type [robot_state_publisher/state_publisher]: Cannot locate node of type [state_publisher] in package [robot_state_publisher]. Make sure file exists in package path and permission is set to executable (chmod +x)
   ```

   Solution: change to the directory: `<path to catkin_ws>/src/franka_ros/franka_visualization/launch`. In this directory there are 2 files: `franka_visualization.launch`, `franka_visualization.rviz`. Then, `chmod +x franka_visualization.launch`, but I do not know whether this step is necessary, for after change its mode, it still does not work. From [this blog](https://blog.csdn.net/weixin_44525754/article/details/113773085), open `franka_visualization.launch` and change the `type ` of `robot_state_publisher` from `state_publisher` to `robot_state_publisher`. That works!

### Running some examples

First:

```bash
cd <path to libfranka>/build/examples
```

- Run `echo_robot_state`:

  ```bash
  echo_robot_state 172.16.0.2
  ```

  Example output:

  ```bash
  {
    "O_T_EE": [0.998578,0.0328747,-0.0417381,0,0.0335224,-0.999317,0.0149157,0,-0.04122,-0.016294,
               -0.999017,0,0.305468,-0.00814133,0.483198,1],
    "O_T_EE_d": [0.998582,0.0329548,-0.041575,0,0.0336027,-0.999313,0.0149824,0,-0.0410535,
                 -0.0163585,-0.999023,0,0.305444,-0.00810967,0.483251,1],
    "F_T_EE": [0.7071,-0.7071,0,0,0.7071,0.7071,0,0,0,0,1,0,0,0,0.1034,1],
    "EE_T_K": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
    "m_ee": 0.73, "F_x_Cee": [-0.01,0,0.03], "I_ee": [0.001,0,0,0,0.0025,0,0,0,0.0017],
    "m_load": 0, "F_x_Cload": [0,0,0], "I_load": [0,0,0,0,0,0,0,0,0],
    "m_total": 0.73, "F_x_Ctotal": [-0.01,0,0.03], "I_total": [0.001,0,0,0,0.0025,0,0,0,0.0017],
    "elbow": [-0.0207622,-1], "elbow_d": [-0.0206678,-1],
    "tau_J": [-0.00359774,-5.08582,0.105732,21.8135,0.63253,2.18121,-0.0481953],
    "tau_J_d": [0,0,0,0,0,0,0],
    "dtau_J": [-54.0161,-18.9808,-64.6899,-64.2609,14.1561,28.5654,-11.1858],
    "q": [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872],
    "dq": [0.00785939,0.00189343,0.00932415,0.0135431,-0.00220327,-0.00492024,0.00213604],
    "q_d": [0.0167347,-0.762775,-0.0206678,-2.34352,-0.0305677,1.53975,0.753862],
    "dq_d": [0,0,0,0,0,0,0],
    "joint_contact": [0,0,0,0,0,0,0], "cartesian_contact": [0,0,0,0,0,0],
    "joint_collision": [0,0,0,0,0,0,0], "cartesian_collision": [0,0,0,0,0,0],
    "tau_ext_hat_filtered": [0.00187271,-0.700316,0.386035,0.0914781,-0.117258,-0.00667777,
                             -0.0252562],
    "O_F_ext_hat_K": [-2.06065,0.45889,-0.150951,-0.482791,-1.39347,0.109695],
    "K_F_ext_hat_K": [-2.03638,-0.529916,0.228266,-0.275938,0.434583,0.0317351],
    "theta": [0.01673,-0.763341,-0.0207471,-2.34041,-0.0304783,1.54006,0.753865],
    "dtheta": [0,0,0,0,0,0,0],
    "current_errors": [], "last_motion_errors": [],
    "control_command_success_rate": 0, "robot_mode": "Idle", "time": 3781435
  }
  Done
  ```

- Run `print_joint_poses 172.16.0.2`

  ```bash
  print_joint_poses 172.16.0.2
  ```

  Example output:

  ```bash
  [0.934972,-0.354722,0,0,0.354722,0.934972,0,0,0,0,1,0,0,0,0.333,1]
  [0.934829,-0.354668,0.0174592,0,0.0163238,-0.00619315,-0.999848,0,0.354722,0.934972,0,0,0,0,0.333,1]
  [0.993249,0.11496,0.015504,0,-0.114847,0.993351,-0.00802793,0,-0.0163238,0.00619315,0.999848,0,-0.00515833,0.00195703,0.648952,1]
  [-0.50603,-0.0654553,-0.860029,0,0.854836,0.094709,-0.510182,0,0.114847,-0.993351,0.00802793,0,0.0767847,0.0114412,0.650231,1]
  [-0.490174,-0.175204,-0.853835,0,-0.170252,0.979966,-0.103347,0,0.854836,0.094709,-0.510182,0,0.446789,0.0532096,0.525273,1]
  [0.980976,0.158626,-0.111908,0,0.0932725,0.120433,0.98833,0,0.170252,-0.979966,0.103347,0,0.446789,0.0532096,0.525273,1]
  [-0.709726,0.704226,-0.0188341,0,0.698276,0.699686,-0.151159,0,-0.0932725,-0.120433,-0.98833,0,0.533115,0.0671686,0.515425,1]
  [-0.709726,0.704226,-0.0188341,0,0.698276,0.699686,-0.151159,0,-0.0932725,-0.120433,-0.98833,0,0.523135,0.0542823,0.409674,1]
  [-0.995598,0.00321012,0.0935672,0,-0.00809594,0.992707,-0.120202,0,-0.0932725,-0.120433,-0.98833,0,0.513491,0.0418295,0.307481,1]
  ```
  
- Run `generate_elbow_motion`

  ```bash
  $ generate_elbow_motion 172.16.0.2
  ```

  Example output1:

  ```bash
  libfranka: Set Joint Impedance command rejected: command not possible in the current mode!
  ```

  *Solution*: press the button so that the blue light changes to white, and turn on the robot again (so that the light turns back to blue).

  Example output 2:

  ```bash
  WARNING: This example will move the robot! Please make sure to have the user stop button at hand!
  Press Enter to continue...
  
  libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]
  control_command_success_rate: 0.488372
  ```

  *Solution*: According to the [documentation](https://frankaemika.github.io/docs/troubleshooting.html#motion-stopped-due-to-discontinuities-or-communication-constraints-violation), the reason is network packet losses.

- Run ` vacuum_object`

  ```bash
  vacuum_object 172.16.0.2
  ```

  Example output1:

  ```bash
  terminate called after throwing an instance of 'franka::NetworkException'
    what():  libfranka: Connection error: Connection refused
  Aborted (core dumped)
  ```

  *Solution*: ?

- 

## Set up Realsense Camera

Now you can control your robot by robot's state. But for more general reinforcement learning, image inputs is necessary. This require us to calibrate our camera.

### Verify `realsense-ros`

At first, verify `realsense-ros` is installed, which should be installed with the Desktop-Full version ROS.

```bash
$ rospack list | grep realsense
realsense2_camera /opt/ros/noetic/share/realsense2_camera
realsense2_description /opt/ros/noetic/share/realsense2_description
```
If `realsense-ros` is not installed:
Install realsense SDK:
```bash
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ sudo apt-get install libudev-dev pkg-config libgtk-3-dev
$ sudo apt-get install libusb-1.0-0-dev pkg-config
$ sudo apt-get install libglfw3-dev
$ sudo apt-get install libssl-dev

$ sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules && udevadm trigger 
$ mkdir build
$ cd build
$ cmake ../ -DBUILD_EXAMPLES=true
$ make
$ sudo make install
```

Also you should check whether your camera can launch:

```bash
$ roslaunch realsense2_camera rs_camera.launch
```

### Install `visp`, `aruco` and `easy_handeye`

`visp` can be installed directly from source:

```bash
$ sudo apt install ros-noetic-visp
$ sudo apt install ros-noetic-vision-visp
```

`aruco` and `easy_handeye` should be made by yourself:

```bash
$ cd <path to catkin_ws>/src
$ git clone https://github.com/IFL-CAMP/easy_handeye
$ git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=<path to libfranka>/build
```

If you meet errors about compatibility of OpenCV, just install the version closest to its requirements. You do not need to uninstall the existing versions.

### Modify the `.launch` file

You can modify the example files in `src/easy_handeye/docs/example_launch/panda_realsense_eyeonbase.launch`. Please read [this issue](https://github.com/IFL-CAMP/easy_handeye/issues/111) first! And remember to change `markerID` and `markerSize`.

After that, copy your modified file into `src/easy_handeye/easy_handeye/launch/` and run

```bash
$ roslaunch panda_realsense_eyeonbase.launch
```

Alternatively,  you can run 

```bash
$ roslaunch easy_handeye panda_realsense_eyeonbase.launch
```

in any directory (this may not work somehow).

Now, several windows are supposed to open. The most important one is xxx. In addition, you'd better see a window showing the image taken by the camera. Maybe this window does not come out automatically. If so, run

```bash
$ rqt
```

Hopefully, you can see a real-time image taken by the camera. If not, in "Plugins->Topics->Topic Monitor", check off the item `aruco_single` (you are expected to check the name in the `.launch` file). After that, every time you launch `easy_handeye`, run in a new terminal:

```bash
$ rqt_image_view
```
