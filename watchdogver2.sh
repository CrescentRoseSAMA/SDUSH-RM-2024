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
# shellcheck disable=SC2034

SEC=1
# 唯一需要修改的地方
BUILD_DIR="/home/ruby/Desktop/work/CV/SDUSH-RM-2024/Detect_For_Jetson/Detect/build"
# 唯二需要修改的地方QWQ
ROOT_PASSWD="qwer"

PROG_NAME="Detect"

UART=(/dev/ttyUSB{0..3})

PROG_ST=$(pgrep -f PROG_NAME | wc -l) # 查看Detect进程是否存在

LOG_DIR="/home/$USER/Documents/AutoAimsLogs"

LOG_NAME="$LOG_DIR/Logs"

ERROR_LOG_NAME="$LOG_DIR/ERROR.log"


# 日志写入
LogIn()
{
    local option=""
    local inform=""
    for param in "$@"
    do
        [ "${param:0:1}" = "-" ] && option="$param"|| inform="$param"
    done 
    case "$option" in
    "-i")
        echo "[INFO]: $inform, [$(date "+%Y-%m-%d %H:%M:%S"])" >> "$LOG_NAME"
    ;;
    "-e")
        echo "[ERROR]: $inform, [$(date "+%Y-%m-%d %H:%M:%S"])" >> "$LOG_NAME"
    ;;
    esac
}

# 创建日志文件
MakeLog()
{

    [ ! -d "$LOG_DIR" ] && mkdir "$LOG_DIR"
    cd "$LOG_DIR"
    # max_log_nums=3，超过3个日志文件删除最早的日志文件
    log_nums=$(ls | wc -l)
    if [ "$log_nums" -gt 3 ]
    then
        while [ "$log_nums" -gt 3 ]
        do
            let log_nums--
            rm "$(ls -rt | head -n 1)"
        done     
    fi
    time_point=$(date "+%Y-%m-%d-%H:%M:%S") # 时间戳
    LOG_NAME="$LOG_NAME$time_point.log"
    touch "$LOG_NAME"
    LogIn -i "Sucessful Create Log, $LOG_NAME"
}


# 启动Detect流程
Start()
{
    declare -i flag=0
    LogIn -i "Waiting for UART Connect"
    # 每隔1s监测一次串口是否连接
    while [ $flag -eq 0 ]
    do  
        for uart in "${UART[@]}"
        do 
            if [ -e "$uart" ]
            then
                flag=${uart:11:11}
                break
            fi  
        done
        sleep $SEC
    done

    # 退出循环说明串口已连接
    echo $ROOT_PASSWD | sudo -S chmod +777 "${UART[$flag]}" # 给串口权限

    LogIn -i "UART Connect to ${UART[$flag]}" # 串口连接信息
    if [ -d "$BUILD_DIR" ]
    then    
        rm -rf "$BUILD_DIR"
    fi
    LogIn -i "Building"
    mkdir "$BUILD_DIR" && cd "$BUILD_DIR" && cmake .. &> /dev/null && make -j8 &> /dev/null
    [ $? -eq 0 ] && LogIn -i "Detect Starting" || LogIn -e "Detect Build Failed" # 启动信息
    [ -e "$ERROR_LOG_NAME" ] && rm "$ERROR_LOG_NAME"
    touch "$ERROR_LOG_NAME"
    ./$PROG_NAME >> /dev/null 2>> "$ERROR_LOG_NAME" & # 启动Detect进程
    LogIn -i "Detect Started"
}

# 启动Detect进程
MakeLog
# 启动Detect进程
Start
# 循环监测Detect进程是否退出
while true
do
    PROG_ST=$(pgrep -f $PROG_NAME | wc -l)
    if [ "$PROG_ST" -eq 0 ]
    then
        ErrInfo=$(cat "$ERROR_LOG_NAME")
        LogIn -e "Detect Aborted, Error Info: $ErrInfo"
        LogIn -i "Detect Try Restarting"
        Start
        sleep $SEC
    else
        sleep $SEC
    fi
done

# 2024-6-24 23:53完工，无bug



