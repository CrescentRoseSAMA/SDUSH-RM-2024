#!/bin/bash

### BEGIN INIT INFO
# Provides:          watchDog
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start watchDog
# Description:       start watchDog
### END INIT INFO

sec=1
cnt=0
PROC_NAME=Detect #进程名字，取决于你自己make后的名称，也可以在终端输入top来查找
Thread=`ps -ef | grep $PROC_NAME | grep -v "grep"` #判断用到，具体用法自行百度
cd /home/jetson/SDUSH-RM-2024/Detect_For_Jetson/Detect/build  #进入文件里面    清除并重新make一下，防止文件损坏（-j提高效率）
FILE=/dev/ttyUSB0
while [ ! -e "$FILE" ]
do
	sleep 0.5
done
echo "qwer" | sudo -S sudo chmod +777 /dev/ttyUSB0 #用自动输入密码并开启ttyusb权限
./Detect #运行

while [ 1 ] #循环，记得大括号里面的1两边都要空格
do
count=`ps -ef | grep $PROC_NAME | grep -v "grep" | wc -l` #//判定进程运行否，是则继续，否则重新启动
echo "Thread count: $count"
if [ $count -gt 1 ];then  #//-gt 大于1情况下 进程没被杀害
	echo "The $PROC_NAME is still alive!"
	sleep $sec
else  #//进程被杀害
	echo "Starting $PROC_NAME"
	cd ~
	while [ ! -e "$FILE" ]
	do
		sleep 0.5
	done

	echo "qwer" | sudo -S sudo chmod +777 /dev/ttyUSB0 #//qwer是我minipc上的密码
	echo "qwer" | sudo -S sudo chmod +777 /dev/ttyUSB1
	cd /home/jetson/SDUSH-RM-2024/Detect_For_Jetson/Detect/build  #//移动到你要编译运行的文件夹里面
	make clean && make -j8
	./Detect
	echo "$PROC_NAME has started!"
		sleep $sec
fi
done

