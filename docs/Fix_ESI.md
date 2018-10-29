修复ESI (EtherCAT Slave Infomation)
====

如果启动elfin_ros_control.launch（或 elfin_ros_control_v2.launch）时遇到类似以下的报错，原因可能是机械臂上的ESI固件错误，也可能是运行的launch文件与机械臂EtherCAT从站版本不一致（详见： [README.md](../README.md)）。
```
ERROR: slave_no(1) : channel(352) is larger than Input bits (256)
```

### 修复ESI固件的方法

打开elfin_ethercat_driver/config/write_esi.yaml文件，对网卡名，希望修复的从站编号以及写入的esi文件进行配置。本软件包提供了两个esi文件: elfin_motor.esi和elfin_motor_v2.esi 。

之后运行以下命令修复ESI
```sh
$ sudo chrt 10 bash
$ roslaunch elfin_ethercat_driver elfin_esi_write.launch
```

最后重启机械臂即可完成修复。

为防误操作，修复后在elfin_ethercat_driver/script/路径下会生成各个从站原ESI的备份。
