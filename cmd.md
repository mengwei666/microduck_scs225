***20261102***
> 泰山派连接命令 ssh root@192.168.1.7 "uname -a"


1. 扫描舵机（不动舵机）
/root/scs225 --port /dev/ttyACM0 --ids 20-24,10-14 scan
我刚才在板子上执行过，10 个全部在线：

id 20 OK version=0x0923
id 21 OK version=0x0923
id 22 OK version=0x0923
id 23 OK version=0x0923
id 24 OK version=0x0923
id 10 OK version=0x0923
id 11 OK version=0x0923
id 12 OK version=0x0923
id 13 OK version=0x0923
id 14 OK version=0x0923
found 10
0x0923 就是 SCS225。

2. 查看舵机详细信息
/root/scs225 --port /dev/ttyACM0 --ids 20-24,10-14 read
会显示位置、电压、温度、负载：

  id    pos    volt  temp   load
  20    513    7.0V   20C      0
  21    509    7.1V   20C      0
  22    525    7.0V   20C      0
  ...
  14    450    7.0V   20C      0
字段含义：

字段	含义
pos	当前位置，0–1023，中位 511
volt	舵机供电电压
temp	温度
load	当前负载


***进行姿态校准命令***

1. 登录板子
ssh root@192.168.1.7
cd /root
2. 卸力，手动摆好姿态
python3 legs_bringup.py release
把机器人摆到你希望上电后保持的姿态。

3. 记录当前姿态
python3 legs_bringup.py calibrate --save
也可以一条命令完成“卸力 → 摆姿态 → 回车确认 → 保存”：

python3 legs_bringup.py calibrate --release --save
执行后会打印每个关节的：

old_home  ->  new_home
并写入：

/root/legs_config.json
如果某个关节显示：

<-- OUTSIDE LIMITS
说明当前姿态超过了之前设置的角度限位，需要先调整姿态或放宽 min_deg/max_deg。

4. 测试保持这个姿态
python3 legs_bringup.py hold --duration 3
机器人会平滑回到刚校准的姿态并保持。Ctrl-C 会释放力矩。

5. 设置上电自动保持
systemctl enable --now legs-hold.service
systemctl status legs-hold.service
查看日志：

journalctl -u legs-hold.service -f
服务会在 /dev/ttyACM0 出现后等待最多 30 秒，然后执行：

python3 /root/legs_bringup.py hold --duration 3
以后每次上电都会自动回到并保持这个姿态。

6. 取消上电保持

systemctl disable --now legs-hold.service
 /root/legs_bringup.py release
7. 注意
不要同时启用：

legs-hold.service
robotd-bench.service
两者会同时往同一条 SCS 总线写目标位置。要用 robotd 跑策略时，先：

systemctl disable --now legs-hold.service
systemctl start robotd-bench.service
当前板子上的状态：

/root/legs_bringup.py      已更新，支持 calibrate/hold/release
/root/legs_config.json     当前默认姿态
legs-hold.service          已安装，disabled
你摆好姿态后执行：

cd /root
python3 legs_bringup.py calibrate --save
python3 legs_bringup.py hold --duration 3
systemctl enable --now legs-hold.service
即可实现“校准这个姿态，上电默认保持”。



24  374 639
23  242 792
22   30 800
21  430 589
20  360 628

14  384 626
13  274 810
12  250 800
11  430 589
10  360 628




python3 legs_bringup.py imu --seconds 5