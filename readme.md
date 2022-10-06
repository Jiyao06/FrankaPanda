**Note**: All sentences starting with **Note** are added by Derick Chen and Jiyao Zhang.


# 1 安装实时内核

## 1.1 下载、解压和验证

### 1.1.1 下载

**去www.kernel.org/pub/linux/kernel里面找url并下载**

共需要4个文件：内核xz，内核sign，补丁xz，补丁sign。内核和补丁的版本号要对应，比如这里的4.14.12

示例：**用浏览器浏览到以下url**

**https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz**

**https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign**

**https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz**

**https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign**

**NOTE**: 如果下载过慢，可以从清华源下载，根目录在**https://mirror.tuna.tsinghua.edu.cn/kernel/**，后续目录与上面保持一致


可以**用`uname -r`查看操作系统发行编号。`uname -a`显示更多信息**

注意事项：补丁只对特定的版本存在，故可能无法完全匹配你的内核版本，但建议下载的版本和你的系统内核版本最接近

*这里的版本号中2比10更接近1（按数字大小，而不是字典序）*

**Note**: if you find it slow to download, try https://mirror.bjtu.edu.cn/kernel/linux/kernel/v5.x/

### 1.1.2 解压与校验

先解压.xz。在刚刚下载到的目录打开终端，执行命令

（注意，你的版本号可能不同。推荐输入`xz -d li`之后按tab补全。之后的各个命令中也要注意修改版本号）

**`xz -d linux-4.14.12.tar.xz`**

**`xz -d patch-4.14.12-rt10.patch.xz`**

`-d`在解压时不保留源文件，所以现在有4个文件，且一个.tar和一个.patch各自有对应的.sign文件

现在校验文件是否完整。尝试执行

`gpg2 --verify linux-4.14.12.tar.sign`

`gpg2`执行各种验证签名，加密解密相关操作

此时由于没有公钥，无法执行，查看错误信息，复制所需的key ID的后八位，在keyserver上得到公钥

`gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x<复制得到的后八位>`

ｄｅｐｒｅｃａｔｅｄ！所以说可能需要网上搜

位数可能也要更多

**Note**: I successfully accessed the public using the following command:

```bash
$ gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 0x<the whole id>
gpg: key xxxxxxxxxxxxxxxx: 1 duplicate signature removed
gpg: key xxxxxxxxxxxxxxxx: public key "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" imported
gpg: Total number processed: 1
gpg:               imported: 1
```



再次尝试校验

`gpg2 --verify linux-4.14.12.tar.sign`

看到“Good Signature”

同样的方法校验另一个文件

`gpg2 --verify patch-4.14.12-rt10.patch.sign`

`gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x<复制得到的后八位>`

`gpg2 --verify patch-4.14.12-rt10.patch.sign`

最后解压.tar

`tar xf linux-4.14.12.tar`

## 1.2 打包安装

### 1.2.1 设置config

`cd linux-4.14.12`

进入解压出的文件夹

`patch -p1 < ../patch-4.14.12-rt10.patch`

打上补丁，其中`../`表示上级目录

`cp -v /boot/config-$(uname -r) .config`

`cp`复制，`-v`输出具体信息，`$(uname -r)`表示把你执行`uname -r`会输出的内容作为字符串放到这个位置

现在把原本系统的config拷贝过来作为新内核的.config，这样能尽可能保持配置不变

*.config不恰当可能会影响功能，比如无线网卡驱动不可用等等*

输出信息示例

`'/boot/config-4.15.0-30-generic' -> '.config'`

现在手动调整.config

`sudo apt-get install build-essential libncurses-dev bison flex libssl-dev libelf-dev zstd dwarves`

安装必要的包

`make menuconfig`

**Note**: when I executed `make menuconfig`, I got

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

So, I just maximize the window size of my terminal.

然后在弹出界面中上下左右和回车操作，找到General Step->preemption model，改成fully使得实时性最强

搜索SYSTEM_TRUSTED_KEYS，确认无内容



### 1.2.2 打包安装

`sudo make -j4 deb-pkg`

**Note**: I have got several errors:

- `dpkg-source: error: unrepresentable changes to source`;

- `dpkg-source: error: cannot represent change to vmlinux-gdb.py: new version is symlink to /home/cdm/Downloads/linux-5.13.19/scripts/gdb/vmlinux-gdb.py old version is nonexistent`;

  *solution*: `rm vmlinux-gdb.py`

- `make[1]: *** No rule to make target 'debian/canonical-revoked-certs.pem', needed by 'certs/x509_revocation_list'.  Stop.` 

  *solution*: `scripts/config --disable SYSTEM_REVOCATION_KEYS`;

- `BTF: .tmp_vmlinux.btf: pahole (pahole) is not available
  Failed to generate BTF for vmlinux
  Try to disable CONFIG_DEBUG_INFO_BTF`
  
  *solution*: `sudo apt-get install dwarves`;

- `/debian/xxxx`
  
  *solution*: `sudo vim .config`，搜索debian，把“”里面的都删除，然后再进行编译（重复上述步骤）

**NOTE**: If you find the kernel is too large, you can remove `CONFIG_DEBUG_INFO=y` in `.config`.

`sudo dpkg -i ../linux-headers-4.14.12-rt10_*.deb ../linux-image-4.14.12-rt10_*.deb`

### 1.2.3 验证安装效果

重启，`uname -a`看到字符串`PREEMPT RT`

`cd /sys/kernel`

`sudo vim realtime`

看到数字`1`



因为同时在CPU Ring0上跑的只能有一个内核

所以得在启动的时候就决定好启动哪个

操作系统是内核态和用户态工具的总称，内核只是一部分

像Linux装多个内核就是用不同的内核启动，但是用户态的东西都是共享的

所以一般认为还是同一个系统（）

# 2 安装显卡驱动

**Note**: 可以参考 **https://blog.csdn.net/weixin_39275295/article/details/119173247** 实现

一重装好系统，马上就要安装显卡驱动，并重启确认不会导致黑屏再进行其它动作，*免得心血白费*

因为某厂垄断已久，态度恶劣，闭源驱动问题很多，兼容性很烂

而且它官网推荐也不上心，官网推荐的驱动用了重启会黑屏，但Ubuntu推荐的不会。*要用Ubuntu推荐的*

**NOTE**: I get several errors when installing the NVIDIA driver:

- 

（顺便一提，它官网cuDNN教程也有坑。害，毕竟人家主要用户是游戏玩家）

## 2.1 禁用nouveau

### 2.1.1 使用vim添加驱动黑名单

**<u>`sudo vim /etc/modprobe.d/blacklist-nouveau.conf`</u>**

`vim`：文本编辑

`~/`, `/`, `./`, `../`, `../../`含义自行百度

值得一提的是：当前目录下的文件如果作为参数可以省略`./`，*但如果直接执行则不能省略，否则会被认为是一个命令*

*gedit可能会有编码方面的问题，导致保存时报错！vim比较稳定*

vim的使用自行百度。主要键位：方向键移动，i插入，Esc退出插入模式，/查找，:wq回车保存并退出，:q回车退出但不保存

**<u>为nouveau添加黑名单</u>**

**<u>`blacklist nouveau`</u>**

**<u>`options nouveau modeset=0`</u>**

**<u>保存退出，并运行以下命令进行更新</u>**

**<u>`sudo update-initramfs -u`</u>**

### 2.1.2 验证禁用

**<u>`lspci | grep nouveau`</u>**

“管道符号”`|`的含义：本来尝试直接`lspci`，会输出一个很长的字符串到终端，而现在直接作为了`grep`的输入

`grep`：查找

*注意这里的输入并不是“参数”。可以尝试`grep 1`回车后`1122`，和`grep 1 1122`完全不同。后者是两个参数，其中`1122`被理解成文件名。实际上，`grep`默认接收终端的“标准输入”，而如果加了参数，就会查看文件*

总之，本来的用法是先`grep nouveau`然后输入字符串。但是现在相当于把`lspci`输出结果作为输入给了`grep nouveau`，也就是在`lspci`输出信息中找nouveau这个子串

此时查找不到nouveau才是对的。现在<u>**重启，再次`lspci | grep nouveau`**</u>确认没有输出



https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_install_apollo_kernel.md#install-nvidia-driver-for-realtime-kernel

没有文件夹就`mkdir`

### 3.5.2 终端代理开关

**<u>在~/.bashrc末尾加入以下代码并更新</u>**

**<u>`alias proxyon="export http_proxy=http://127.0.0.1:12333 && export https_proxy=http://127.0.0.1:12333 && env | grep proxy | xargs echo"`</u>**

**<u>`alias proxyoff="unset http_proxy https_proxy all_proxy no_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY && env | grep proxy | xargs echo && env | grep PROXY | xargs echo"`</u>**

`alias`：设置命令的别名，以后就可以用别名执行一串命令

`&&`：成功则继续执行下一条命令

`env`输出环境变量列表，通过`|`给`grep`查找含有proxy的，然后再`| xargs`给`echo`输出

其中注意，`|`只能把输出变为输入，不能变为参数。所以对于`echo`，想要成功输出就必须加上`xargs`

`unset`：清除环境变量

双引号中有单引号不会造成识别的问题，不需要转义

备注：如果`proxyoff`的定义是只删除http_proxy和https_proxy，环境里有时因暂时未知的原因（待考证）还会残留no_proxy, all_proxy等变量，这也会导致错误。所以这里定义时索性把所有八个变量一起`unset`

此时在终端处可以用命令`proxyon`和`proxyoff`开关代理

# 4 安装CUDA和cuDNN

备注：456可以同步进行，节省等待时间

## 4.1 文件下载

### 4.1.1 CUDA

<u>**上https://developer.nvidia.com，按搜索按钮搜索想要的CUDA版本，比如CUDA 11.1（照顾ZED相机），点进链接，一路选择Linux->x86_64->Ubuntu->18.04->runfile [local]，复制那个`wget`命令**</u>（`wget`用来下载网络资源）



要先明确ZED SDK需要的版本





<u>**在你想下载文件的目录（我们以`~/下载`为例）打开终端，运行那个`wget`命令，把CUDA资源下载到该目录**</u>

### 4.1.2 cuDNN

**<u>搜索cuDNN Download，去NVIDIA官网，用邮箱注册一个账号，然后Verify，登录（不要用微信之类的登录）</u>**

**<u>有个cuDNN Download Survey，不填也行</u>**

**<u>然后选cuDNN的合适版本（注意查看系统，CUDA版本等），用浏览器下载一个.tgz</u>**

现在你应该有一个cuda开头的.run，一个cudnn开头的.tgz

## 4.2 安装CUDA

### 4.2.1 安装

**<u>在上面的终端中</u>**

**<u>`sudo sh cuda*.run`</u>**

`sh`：运行脚本



<u>**一路继续，直至选择安装组件，把Driver前面的X按Enter去掉（因为驱动已经有了），再选Install**</u>

安装结束，尝试

**<u>`cat /usr/local/cuda/version.txt`</u>**

可以输出文件内容，看到版本

如果是卸载旧的，需要重启！！

### 4.2.2 添加环境变量

<u>**用vim在~/.bashrc末尾加上**</u>（*内容跟CUDA版本有关，如果版本等不同，不要直接复制*）

**<u>`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64`</u>**

**<u>`export PATH=$PATH:/usr/local/cuda-10.2/bin`</u>**

**<u>`export CUDA_HOME=$CUDA_HOME:/usr/local/cuda-10.2`</u>**

**<u>并更新（回忆：`. ~/.bashrc`）</u>**

**<u>`nvcc -V`</u>**

看到版本号

**<u>`cd /usr/local/cuda-10.2/extras/demo_suite`</u>**

转到demo所在文件夹（上述命令具体写什么和CUDA版本有关）

**<u>`./deviceQuery`</u>**

运行指定程序，看到输出

## 4.3 安装cuDNN

**<u>解压刚刚的cudnn开头的.tgz</u>**

**<u>解压出的cuda文件夹中有lib64和include两个文件夹，在这个cuda文件夹中打开终端</u>**

**<u>`sudo cp include/cudnn*.h /usr/local/cuda/include`</u>**

**<u>`sudo cp -P lib64/libcudnn* /usr/local/cuda/lib64`</u>**

`cp`：复制。`-P`：“保持”，具体百度

**<u>`sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*`</u>**

`chmod`：赋予权限

**<u>`cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2`</u>**

在那个.h文件中查找字符串从而看到版本输出

`-A 2`：输出之后2行



# 5 安装Anaconda和pytorch

为了ROS顺利运行我们安装anaconda2

## 5.1 Anaconda下载安装

**<u>在”下载“目录终端打开</u>**

**<u>`wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/Anaconda2-5.3.1-Linux-x86_64.sh`</u>**

**<u>`sh Anaconda2-5.3.1-Linux-x86_64.sh`</u>**

<u>**一路Enter和yes**</u>（最后vscode装不成功就不管了，没事）

<u>**`. ~/.bashrc`**</u>

*注意Anaconda会往~/.bashrc里写入东西，需要手动更新*

## 5.2 Anaconda使用

### 5.2.1 进入base

**<u>`conda activate base`</u>**

**<u>`echo $PATH`</u>**

可以看到anaconda增加了环境变量，来改变运行的环境

（这时如果`conda deactivate`出去可以发现环境变量少了）

<u>**`python`**</u>

进入python，看到是python2

**<u>`quit()`</u>**

### 5.2.2 加清华源

**<u>`conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/`</u>**

**<u>`conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/`</u>**

**<u>`conda config --set show_channel_urls yes`</u>**

### 5.2.3 创建py3环境

**<u>`conda create -n py3_env python=3.6`</u>**

**<u>`conda activate py3_env`</u>**

**<u>`echo $PATH`</u>**

可以看到环境变量变了

**<u>`python`</u>**

是python3

**<u>`quit()`</u>**

## 5.3 pytorch

<u>**浏览器打开 https://pytorch.org/get-started/locally/**</u>

<u>**选择相应选项，复制命令到刚才的终端运行**</u> *（此时保证终端左侧显示(py3_env)）*

https://www.jb51.net/article/184036.htm

pytorch的和电脑主体CUDA不一样

pytorch编译和跑可以用两个（看文章）

# 6 安装ROS

## 6.1 主体操作

**<u>上http://wiki.ros.org/melodic/Installation/Ubuntu#Installation，对着不断输入命令即可</u>**

**<u>`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`</u>**

这步加源

`sh -c`执行脚本，将字符串（单引号括起）作为完整的命令，这样`sudo`的影响范围是整条命令

`echo 字符串 > 文件`：把双引号中的输出重定向到文件（会替换文件内容成指定字符串）

回忆：单引号中双引号不需要转义

（如果是`>>`，就是“追加重定向”，也就是添在文件后面，在修改`~/.bashrc`时常用）

而写入内容是`deb http://packages.ros.org/ros/ubuntu bionic main`，就是要加的源

其中bionic是ubuntu版本名称，被`$(lsb_release -sc)`取得作为字符串一部分。可以在终端中尝试`lsb_release -sc`看输出结果

<u>**`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`**</u>

这步加公钥，从keyserver上找公钥

`apt-key`管理公钥，`adv`修改高级设置

**<u>`sudo apt update`</u>**

*加入新的源了之后，一定要update一下，否则`apt install`会缺包*

加源，加公钥，update是安装软件常见的前置步骤。之后还会反复看到，只不过各个官网写法略有差别

**<u>`sudo apt install ros-melodic-desktop-full`</u>**

**<u>用vim在~/.bashrc末尾加入</u>**

**<u>`source /opt/ros/melodic/setup.bash`</u>**

**<u>并更新（回忆：`. ~/.bashrc`）</u>**

**<u>`sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`</u>**

**<u>`sudo apt install python-rosdep`</u>**

## 6.2 最后两个命令

网速慢的时候，最后两个命令很难成功

**NOTE**: I found the network very good in the libarary of PKU. I finished these two command only in a few seconds.

### 6.2.1 确保代理正常

<u>**首先要保证代理能用，可以使用定义过的`proxyon`确认**</u>

### 6.2.2 修改超时阈值

**<u>依次打开三个文件，每次都用vim查找DOWN，并将该行中数字15改大</u>**

**<u>`sudo vim /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py `</u>**

**<u>`sudo vim /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py`</u>**

**<u>`sudo vim /usr/lib/python2.7/dist-packages/rosdep2/rep3.py`</u>**

**<u>然后再尝试</u>**

**<u>`sudo rosdep init`</u>**

**<u>`rosdep update`</u>**

尝试多次直至成功

### 6.2.3 尝试验证

**<u>`roscore &`</u>**

**<u>按Ctrl+C</u>**

**<u>`rqt_graph`</u>**

**<u>去掉Hide处所有勾选，看能否正常看到/rosout</u>**

**<u>终端按Ctrl+C</u>**









# 8 franka机械臂相关配置

## 8.1 libfranka和franka_ros

手动安装libfranka（官网）

minimum-CPU

installation

https://frankaemika.github.io/docs/installation_linux.html

DFranka_DIR:PATH要对应

getting started

 jitter test



关注手动安装中的git checkout命令（之后还会提到

https://unix.stackexchange.com/questions/10814/what-is-the-usr-local-src-folder-meant-for

手动装libfranka的推荐位置(和franka_ros不一样)

注意如此一来 franka_ros装的时候需要指定参数-DFranka啥啥（这就体现遵守FHS重要性了！）



自动则`roscd libfranka`

验证方法也不同

https://frankaemika.github.io/docs/franka_ros.html#ros-visualization

![image-20210507112532738](/home/a/.config/Typora/typora-user-images/image-20210507112532738.png)



坑：

所有涉及操作，要加sudo（和实时性有关）

![image-20210507112118358](/home/a/.config/Typora/typora-user-images/image-20210507112118358.png)

这样就不用（）

实际上，roslaunch不能用sudo，所以必须要加这个

## 7.1 尝试施教

**涉及真机，务必在指导下操作**

纸飞机关掉代理，然后打开机械臂，至预热好（黄灯不闪）**没预热好，是连不上robot.franka.de的**

浏览器设置代理一停全停 终端可以开两个一个`proxyon`一个`proxyoff`

有线网连接台式机和franka机械臂底座，浏览器访问robot.franka.de

找学长要账号密码，登录

右边Joints，开锁，机械臂蓝灯亮

左下tasks->加号，创建一个task

拖动一个CART MOTION到左上角加号，点击它

按下**实体**黑色按钮，机械臂白灯亮，然后用手施教，尝试Add，构建出一个CART MOTION序列

一路Done和Continuehttp://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

拔起黑色按钮，机械臂蓝灯亮。点击播放键，机械臂绿灯亮，开始运动

右边Joints，关锁，机械臂黄灯亮

右上角两杠->User Manual，下载用户手册

右上角两杠->Shut down，直至显示Finished shutting down，成功关机



关机不能马上开？？？？





手动动gripper之后要homing！！！(网页desk里面）

https://github.com/frankaemika/franka_ros/issues/16

尽量不要手动动，要不然rviz出问题



timeshift要重启机械臂

重启电脑要homing

反正遇事不决重启



# py2_test



http://wiki.ros.org/ROS/Tutorials/CreatingPackage

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

https://answers.ros.org/question/251292/why-to-use-source-develsetupbash/

否则自己的包认不到

可以`echo $ROS_PACKAGE_PATH`查看

可以加到~/.bashrc

`source ~/catkin_ws/devel/setup.bash`





`roscore &`

Ctrl+C

确定左侧环境 没有括号

再运行py2_test的脚本









py3

https://www.cnblogs.com/h46incon/p/6207145.html

进入环境，然后安装

`pip install catkin-tools`

`pip install rospkg`

看activate之后的`$PATH`是啥

进去这个目录看到python

蓝色 绿色的含义

https://zhidao.baidu.com/question/579632338.html

试着直接运行解释器 方便理解

![image-20210507032619068](/home/a/.config/Typora/typora-user-images/image-20210507032619068.png)

不需要改shebang，只用开conda环境



![image-20210508093000675](/home/a/.config/Typora/typora-user-images/image-20210508093000675.png)

![image-20210508094025815](/home/a/.config/Typora/typora-user-images/image-20210508094025815.png)



前面都说错了

shebang还是有用的，不信，你没shebang，直接当成shell脚本

env的是“相对”，看你左边有没有括号

也可以绝对写

建议都相对写，然后开不同环境

![image-20210508095420147](/home/a/.config/Typora/typora-user-images/image-20210508095420147.png)

这个是绝对

rosrun 等价于直接./**.py

roslaunch就是打包rosrun





一些缺包该装什么？CSDN

https://www.cnblogs.com/zmm1996/p/10775283.html





. devel/setup.bash --extend

# ZED

cuda版本要对！

要代理（上网和安装都要）

https://www.stereolabs.com/docs/installation/linux/

https://www.stereolabs.com/docs/ros/

![image-20210508011739139](/home/a/.config/Typora/typora-user-images/image-20210508011739139.png)

`conda list | grep pyzed`

![image-20210508015024131](/home/a/.config/Typora/typora-user-images/image-20210508015024131.png)

先确保这里能用

USB3.0，flip typeC



`roslaunch zed_display_rviz display_zedm.launch`

这个和zed_wrapper那个不能launch不能同时

如果和franka同时需要base_frame



想同时rviz？

 `rosrun rviz rviz `

手动加



 rqt_image_view 

这个比rviz大！舒服

也是个节点





`rosrun image_view extract_images _sec_per_frame:=1 image:=/zedm/zed_node/rgb/image_rect_color`

![image-20210508070352575](/home/a/.config/Typora/typora-user-images/image-20210508070352575.png)



最终效果

![image-20210508215315422](/home/a/.config/Typora/typora-user-images/image-20210508215315422.png)













标定

标定板 边长，内部角点个数

关掉ros的zedm_wrapper，然后

https://support.stereolabs.com/hc/en-us/articles/360011828773-How-do-I-recalibrate-my-ZED-stereo-camera-

![image-20210508075926581](/home/a/.config/Typora/typora-user-images/image-20210508075926581.png)





# 7 CycleGAN训练和使用

https://github.com/junyanz/pytorch-CycleGAN-and-pix2pix

注意直接搜CycleGAN是lua版本的



src文件夹的意义（联系ros下载到src）



https://blog.csdn.net/xqlily/article/details/104769042

`jsonpatch`

`pip install visdom`

`pip`才有 `conda`没有

conda有的时候不够全

`visdom`能用了

![image-20210507094546594](/home/a/.config/Typora/typora-user-images/image-20210507094546594.png)

这是端口冲突解决方法

（之前没正常关）



看图：

重新启动`visdom`和训练程序

`python -m visdom.server`

另一个终端

`python train.py --dataroot ./datasets/maps --name maps_cyclegan --model cycle_gan --display_freq 5 --update_html_freq 40`

具体数据集名字：map

test参数也去注释找（map）



看显卡:`watch -n 1 nvidia-smi`







pycharm

https://www.jetbrains.com/pycharm/download/download-thanks.html?platform=linux&code=PCC

下载下来，里面有教程

加环境变量，就可以以后pycharm.sh直接启动

第一次开可能略卡

看doc代码结构



pychram.sh 某某.py，也可以



选解释器，available for all proj

还有加参数（直接粘贴）



PYTHONPATH is an environment variable which you can set to add  additional directories where python will look for modules and packages.  For most installations, you should not set these variables since they  are not needed for Python to run. Python knows where to find its  standard library.

The only reason to set PYTHONPATH is to maintain directories of custom Python libraries that you do not want to install  in the global default location (i.e., the site-packages directory).



a@a:~/Downloads/pytorch-CycleGAN-and-pix2pix$ echo $PYTHONPATH
/home/a/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages

https://stackoverflow.com/questions/21199382/why-does-pycharm-always-add-contents-root-to-my-pythonpath-and-what-pythonpat

不用担心，不会改变系统的环境变量，只是pycharm自己用





加红色断点，debug(shift +f9)，逐步

F8

shift+f8

f7







# 接口

tip：vim和gedit：gedit可能有空格和tab的问题



catkin造新包，加个script，更新CMakelist，更新bashrc，chmod +x

![image-20210508083558460](/home/a/.config/Typora/typora-user-images/image-20210508083558460.png)

![image-20210508092034968](/home/a/.config/Typora/typora-user-images/image-20210508092034968.png)

我去 竟然是这样



代码：在sub_image里面

**还电脑之前，要拷一下catkin自己的包**



mpi4py使用

`sudo apt install python-pip`

`pip install mpi4py`

https://zhuanlan.zhihu.com/p/25332041

https://mpitutorial.com/tutorials/introduction-to-groups-and-communicators/zh_cn/

py3_env里也装一下然后用mpi4py

（不过上面的是并行计算，用不了



我们现在要串行计算

https://blog.csdn.net/ljhandlwt/article/details/51980267



vim 一个打开 一个试运行

`:w`即可

非常方便

![image-20210509044811408](/home/a/.config/Typora/typora-user-images/image-20210509044811408.png)

https://stackoverflow.com/questions/9970409/mpi-spawn-root-process-does-not-communicate-to-child-processes

上面是c，python自己找



传hello可以，numpy不行

https://bitbucket.org/mpi4py/mpi4py/issues/102/unpicklingerror-on-commrecv-after-iprobe

`import mpi4py`之后还要单独import`mpi4py.MPI`？？？

作者给的不管用



numpy转list开销很大！！！（实验：是传输是瓶颈还是转换？）

创建了新的list.

这不好



![image-20210509093816632](/home/a/.config/Typora/typora-user-images/image-20210509093816632.png)

根本不是mpi4py问题，而是pickle问题。而且到这一步已经接近解决

关键是怎么想到实验测出不是mpi4py问题



https://docs.python.org/2/library/pickle.html#

https://docs.python.org/2/library/stringio.html#module-StringIO

注意上面这篇文章的memory file

https://python3-cookbook.readthedocs.io/zh_CN/latest/c05/p21_serializing_python_objects.html





### 编码相关



### 0

Unicode 和 UTF-8 有什么区别？ - 邱昊宇的回答 - 知乎 https://www.zhihu.com/question/23374078/answer/24385963

Unicode是字符集，集合元素像是U+某某

UTF-8和其他的编码规则都是把U+某某转化为字节序列

解码是把字节序列转化为U+某某



Unicode和UTF-8一个是集合，一个是映射



广义的Unicode包括字符集（狭义）Unicode和UTF-某等等编码规则，也就是包含了集合和相应映射



### 1

https://zhuanlan.zhihu.com/p/25272901

https://blog.csdn.net/yanghuan313/article/details/63262477

讲的很清楚



关键点：

```
>>> '\xe6\xb5\x8b' == '测'
True
>>> print('\xe6\xb5\x8b')
测
```

python2的str类型是字节序列

如果python3拿到这个字节序列，

自动解码成str类型（这已经不太合理）

这时直接pickle不允许loads，需要转化为bytes类型

但是这里转换成bytes之后，如果再解码就会炸，比如出现0xff（pickle的锅，需要指定encoding='bytes'）









='bytes'
perfect_fourth
10:40 AM
暴力美学
嗷嗷搞定了
虽然我还是不清楚原因
为什么其他任何都会出现0xff
utf-8 ascii latin1
crypotatographer
10:42 AM
因为py3的字符串是有编码的
py2的字符串是字节数组
py3不知道py2的字符串是怎么来的，就强行解码
perfect_fourth
10:43 AM
py2字符串是字节数组，换句话说，不能被ascii或者utf8或者等等一些解码
crypotatographer
10:43 AM
除非强制指定encoding让它保留字节类型，不然解码了就会炸（因为原来根本就不是字符序列）
perfect_fourth
10:43 AM
只能暴力读字节
crypotatographer
10:43 AM
对，除非py2那边是对应的字符序列，就能被解码
perfect_fourth
10:44 AM
什么叫py2那边是对应的字符序列





我查查
crypotatographer
10:46 AM
例如py2那边是"abc"，编码成bytes之后是三个ascii可表示的byte，那么py3这边解码出来还是三个ascii字符，就对上了
但是更复杂的字符集就会有很多麻烦
perfect_fourth
10:47 AM
也就是bytes和ascii, utf8等也没有本质区别，本质也是一套编码
有些时候，恰好两种编码结果是一样的

对，bytes可以算是identity编码
别的就是一个字符对应某种bytes
perfect_fourth
10:48 AM
所以又出现了新的疑惑
python2的中文怎么编码
crypotatographer
10:48 AM
取决于编译设置
这就 很烦
在win上通常是本地代码页（臭名昭著GB2312）
perfect_fourth
10:49 AM
那么#-*- coding
crypotatographer
10:49 AM
在lin上看发行版
perfect_fourth
10:49 AM
utf8
跟刚才说的都是bytes
这两个是何关系[Facepalm]
如果我在py 2.x里面开头加# -*-
crypotatographer
10:50 AM
嗷，shebang决定python怎么解码当前文件

https://www.runoob.com/python/python-chinese-encoding.html
perfect_fourth
10:51 AM
我草这个也称为shebang？？
bang都没有233
crypotatographer
10:53 AM
草，叫习惯了，严格来说确实不叫shebang
perfect_fourth
10:54 AM
这时py3如果想要读取中文 可就麻烦了
直接读取bytes得到一堆被utf8编码过的
[Facepalm]
crypotatographer
10:55 AM
对，但是py3的load的默认设置是encoding='utf8'
所以它会尝试自动转回中文（或者什么奇怪文）
但是你遇到的问题是它根本就不是字符
perfect_fourth
10:56 AM
对 0xff
所以说
如果py3使用非bytes，大概率读到0xff然后爆掉
如果py3使用bytes，可能读到非ascii然后尝试自动用utf8
然而如果py2送一个非utf8，就尴尬了，py3只能使用bytes而不能手动指定gbk啥的
要命
crypotatographer
10:58 AM
可以手动指定的啊
但是就只能在
读取解开的对象的时候
手动str.decode('gbk')
算是一个坑吧
perfect_fourth
10:59 AM
但是刚刚又只能bytes,,
我想象
crypotatographer
11:00 AM
这个是解开之后再对单独的字符串解码
不是整个改编码
perfect_fourth
11:01 AM
准备去复习ics
crypotatographer
11:01 AM
就是 你发现编码不对再一个一个修 就很痛苦
perfect_fourth
11:01 AM
感觉没法一下说清楚
不过感觉有个大概的问题清单
我再去查
crypotatographer
11:02 AM



![image-20210509122529038](/home/a/.config/Typora/typora-user-images/image-20210509122529038.png)





???
到底怎么接受py2传来的utf-8字符串？gbk字符串？

？？？

亮 频繁 怎么回事





`catkin_make -DPYTHON_EXECUTABLE=/home/a/anaconda3/envs/py3_env/bin/python3 -DPYTHON_INCLUDE_DIR=/home/a/anaconda3/envs/py3_env/include/python3.6m -DPYTHON_LIBRARY=/home/a/anaconda3/envs/py3_env/lib/libpython3.6m.so`

注意anaconda这里lib后面没有x86-64啥啥，自己看看就行

cv_bridge用py3编译

https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3

默认git下载的版本太高，是py3.7的，所以会出问题，我们用py3.6的话，就git checkout 1.12.8



直接catkin_make不行（可以看信息，还是用python2）（能编译成功，但是最后还是py2）

应该catkin_make -D加参数

或者那个链接也提供了第二种方法



opencv安装卡？：

`pip3 install --upgrade pip`

包名是pip，命令pip3（也不总是，conda环境中命令就是pip）

命令不本质（可以软连接，包名本质



下次试着：改成用conda里的

catkin_make -DPYTHON_EXECUTABLE=/home/a/anaconda3/envs/py3_env/bin/python -DPYTHON_INCLUDE_DIR=/home/a/anaconda3/envs/py3_env/include/python3.6m -DPYTHON_LIBRARY=/home/a/anaconda3/envs/py3_env/lib/libpython3.6m.so



(py3_env) a@a:~/anaconda3/envs$ find ./ -name libpython3.6m.so
./thor-rearrange/lib/libpython3.6m.so
./py3_env/lib/libpython3.6m.so

有点小坑（conda的lib那个和网上的不严格对应



多个空间工作确保环境变量

https://blog.csdn.net/lclfans1983/article/details/107453043

![image-20210510055718606](/home/a/.config/Typora/typora-user-images/image-20210510055718606.png)



```shell
(base) a@a:~$ echo $ROS_PACKAGE_PATH 
/opt/ros/melodic/share
(base) a@a:~$ echo $CMAKE_PREFIX_PATH 
/opt/ros/melodic
(base) a@a:~$ . ./catkin_ws/devel/setup.bash 
(base) a@a:~$ echo $ROS_PACKAGE_PATH 
/home/a/catkin_ws_py3/src:/home/a/catkin_ws/src:/opt/ros/melodic/share

```

ros_package_path要对

每次要source，要不然找不到包

具体，读源码





???自己create的包为啥有时需要roscd一次（手动输入全名）只能才能tab补全



有时出问题可以手动管理环境变量pythonpath

```
Python 3.6.13 |Anaconda, Inc.| (default, Feb 23 2021, 21:15:04) 
[GCC 7.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import torch
>>> torch.__file__
'/home/a/.local/lib/python3.6/site-packages/torch/__init__.py'

```

export PYTHONPATH=$PYTHONPATH:/home/a/.local/lib/python3.6/site-packages

https://www.cnblogs.com/qingdou/p/11665894.html



https://www.cnblogs.com/saolv/p/7808347.html

了解相关环境变量



python找包坑：根据顺序(可以看path.sys)，找到自己

# Moveit

autoremove remove purge区别

慎用！！





http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html

照着搞

http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

（后面这个链接也可以连真的机械臂）



roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2

这个可以启动控制器

rviz里save config就可以自动add和改planning group，否则要手动调

panel里改视图

option改速度，要不然太大了



出了问题就要重启信息发送器和控制器，否则一直不给用

一个实际运行问题：yaw太大，导致关节到limit，然后后面一直失败

一个python语法坑：f(*(1,2), 3)不行



看keybase里面“新计划”之前的文件

## server和client

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

xml两个depend

txt：REQUIRED

message_files

service_files

generate_messages

CATKIN_DEPENDS

http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient

几个tutorial都看

注意如果忘了修改xml或者txt，报错会导致需要重新source才能找到包



注意import包名.srv之前要先source（然后echo $PYTHONPATH确保有相应环境变量



wait_for_message方法（代替subsc，不需要callback，直接信息传给变量



srv文件命名必须是标识符，不能数字开头

如果数字开头，报错，手动去删，再重新make（ros不成熟，不会自己删）



跨包：要改txt和xml包名部分



https://blog.csdn.net/xiongchun11/article/details/78832947?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromMachineLearnPai2%7Edefault-1.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromMachineLearnPai2%7Edefault-1.control

:set ff回车，可以看到格式是不是unix





一个坑 时间限制（timeout，上网查，改xml





0.04

0.01

出现

[ INFO] [1621429402.658684503]: ABORTED: Solution found but controller failed during execution

但是0.04 0.00就没有问题？？？？

从0循环到0.04也没问题 就离谱



小技巧不同终端开不同ws



msg和srv修改后要重新编译

回忆：涉及python3的编译，需要指定参数



曾经以为的

srv小坑：官网代码不是绝对路径，每次必须cd到指定地方才能找到包名.srv

诶，不是诶，其实找的不是当前路径下的sub_image。可以设法读一下到底读的是哪里的包



tf问题？看xml，然后launch时加参数base_frame

rviz里TF报黄 但是没事。反正现在不需要标定





rosnode cleanup 删除红色节点（红色节点来源？？？）





对坐标轴对象 对单位

明确给的是哪个东西（panda_link8，而不是最低点！！）

所以你0.0肯定撞



看TF原点

看红是x，右手坐标系

![image-20210520075708138](/home/a/.config/Typora/typora-user-images/image-20210520075708138.png)

打勾，看需要的坐标系名字

rosrun rqt_tf_tree rqt_tf_tree     

http://wiki.ros.org/tf#tf_echo

![image-20210520081510423](/home/a/.config/Typora/typora-user-images/image-20210520081510423.png)







# Grasp, actions servers

grasp

**https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py**

注意，epsilon意义，用法（roscd franka_gripper，查msg，注意msg的构造方法

夹紧才是成功

https://frankaemika.github.io/docs/franka_ros.html#franka-hw

看hw和gripper这些官方

发了新代码





http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29

actionlib不在主体tutorial

必须手动wait_for_sever才能等，否则自动覆盖









逐步运行代码，发现有cpp部分，pycharm逐步看不到

vscode试试（vscode改解释器，逐步怎么做？

注意不要多开了。要及时停止（比较pychram

注意F5按的时候是哪个文件（比较pycharm

(base) a@a:~/catkin_ws/src/panda_moveit_config/launch$ vim panda_control_moveit_rviz.launch 
(base) a@a:~/catkin_ws/src/panda_moveit_config/launch$ roscd franka_control/
(base) a@a:/opt/ros/melodic/share/franka_control$ ls
cmake  config  franka_controller_plugins.xml  launch  package.xml
(base) a@a:/opt/ros/melodic/share/franka_control$ cd launch/
(base) a@a:/opt/ros/melodic/share/franka_control/launch$ ls
franka_combined_control.launch  franka_control.launch
(base) a@a:/opt/ros/melodic/share/franka_control/launch$ vim franka_control.launch 

尝试去launch文件找

https://blog.csdn.net/qq_37082966/article/details/99681023?utm_medium=distribute.pc_relevant_download.none-task-blog-2~default~BlogCommendFromBaidu~default-2.nonecase&depth_1-utm_source=distribute.pc_relevant_download.none-task-blog-2~default~BlogCommendFromBaidu~default-2.nonecas

找了几层，然后知道type是执行的程序！

知道了要去找franka_gripper_node

https://blog.csdn.net/qq_38800089/article/details/108363065

找到了可执行程序的位置

不过这里没有源码

上网找源码

（如果你build from source，就有





http://wiki.ros.org/franka_gripper

https://github.com/frankaemika/franka_ros/tree/melodic-devel

选branch

https://github.com/frankaemika/franka_ros/blob/melodic-devel/franka_gripper/src/franka_gripper.cpp

找关键的类franka::Gripper

https://github.com/frankaemika/franka_ros/blob/melodic-devel/franka_gripper/include/franka_gripper/franka_gripper.h

发现这里include了libfranka的

https://github.com/frankaemika/libfranka/tree/master/include/franka

看到gripper.h的函数原型，然后找

https://github.com/frankaemika/libfranka/blob/master/src/gripper.cpp

神奇 epsilon没有用？？





https://www.cnblogs.com/xiel/p/3613919.html

https://blog.csdn.net/guotianqing/article/details/104224439



https://www.baidu.com/link?url=hPoHWIx4B0UzLyduvnZjiLFUWMoHKqyaYvJyAQ56EZd1QJBtowdYH4p5-rsOaK750LC9L11-9M2IuSPpXGe3ZUvSa4tDyC0lnPjyq3qFQ43&wd=&eqid=ea0f292c0029fdea0000000660a8f939



暴力找到include位置

sudo find / -name *resear*interf*
find: ‘/run/user/1000/keybase/kbfs’: Permission denied
find: ‘/run/user/1000/gvfs’: Permission denied
/timeshift/snapshots/2021-05-22_04-00-01/localhost/home/a/libfranka/common/include/research_interface



然而最后是tcp，传过去了，就很麻烦









rostopic list | grep grip

http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics

回忆rostopic echo和pub

rosnode info /move_the_arm_18234_1621688524674 

rostopic echo /franka_gripper/move/goal

发现没有epsilon

发现好像传的都是move不是grasp



pycharm逐步（双击展开技巧！！

![image-20210522093140692](/home/a/.config/Typora/typora-user-images/image-20210522093140692.png)

畅爽

![image-20210522093551875](/home/a/.config/Typora/typora-user-images/image-20210522093551875.png)

第一个conn的notify()那里做了动作

好了，确认了，是client写错了

```
self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
```

这个应该是grasp



注意普通server的返回值可以用（我写了520）

### 修实际问题

注意关了还关会出问题

所以要引入STAY状态（不能只有OPEN和CLOSE）





常见中途停下：关节到极限（要手挪回去）

https://blog.csdn.net/ssw_1990/article/details/104050057

![image-20210523044617149](/home/a/.config/Typora/typora-user-images/image-20210523044617149.png)

手动改严

实验：转1，转7，一个到硬件极限，一个不到





大桌子拼起来，图片干净！！

临时固定相机

要求不拍底座，不拍无关物体，看到手指

opencv裁剪一下

cv2.imshow

cv2.waitKey



TOLERANCE问题：按下拔起，重启roslaunch panda_moveit_config那个（不用手挪）

就是误差太大了。常见成因：抓的东西撞地，导致误差

仔细用rviz看tf，发现panda_EE并不是最低点，而是还差一些！！

而且有的时候会过一点点再刹回来

手动实验：强行不给它运动到位，造成了该错误！

声音不对劲！！

直接撞地也会导致这种

首先，注意panda_EE有点误差，手动发命令测算，把OFFSET改大成0.120

![image-20210523030247766](/home/a/.config/Typora/typora-user-images/image-20210523030247766.png)

加一块地面 防止乱规划

要-0.1，要不然下不去

加一块墙，防止撞相机

https://www.jianshu.com/p/de156969c64f

代码添加障碍物！

https://blog.csdn.net/xudesheng1234/article/details/111224465

![image-20210525035442731](/home/a/.config/Typora/typora-user-images/image-20210525035442731.png)

改碰撞箱，要不然-0.09都会撞

拷进去，然后改xacro

hand和arm都要改

球？保护？



![image-20210525043424328](/home/a/.config/Typora/typora-user-images/image-20210525043424328.png)

这个把publish当service用了……

实际上publish没有保证subscriber在……



















TOLERANCE暂停问题

不能set_goal_tolerance，否则只是增加误差，没有解决问题

clone下来

https://github.com/ros-planning/moveit

暴力grep -r，发现

![image-20210523091813993](/home/a/.config/Typora/typora-user-images/image-20210523091813993.png)

![image-20210523092208968](/home/a/.config/Typora/typora-user-images/image-20210523092208968.png)



![image-20210525044810009](/home/a/.config/Typora/typora-user-images/image-20210525044810009.png)

如何出错了自动恢复？

libfranka和frankaros文档里搜索recovery

https://frankaemika.github.io/docs/franka_ros.html

![image-20210525051239844](/home/a/.config/Typora/typora-user-images/image-20210525051239844.png)

这是个1（回忆tutorial），不是l

最后写shell函数，搭环境完成

gnome-terminal用法





# 第一次进军cpp

## gcc

![image-20210523095400243](/home/a/.config/Typora/typora-user-images/image-20210523095400243.png)

https://jingyan.baidu.com/article/acf728fd7a6db0f8e410a345.html

https://www.runoob.com/w3cnote/gcc-parameter-detail.html

.o是什么？链接是什么？

https://blog.csdn.net/weixin_41143631/article/details/81221777

什么是动态链接

https://www.jianshu.com/p/cdb5cfcb5056

## make

https://www.cnblogs.com/jerrybaby/p/6130574.html

注意include<>和“”区别

https://www.cnblogs.com/hazir/p/linux_make_examples.html

## cmake







http://wiki.ros.org/catkin/commands/catkin_make

注意白名单

