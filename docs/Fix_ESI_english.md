Fix ESI (EtherCAT Slave Infomation)
====

If you got an error info as follows, you may need to fix ESI on the robot.
```
ERROR: slave_no(1) : channel(352) is larger than Input bits (256)
```

### How to fix ESI

Open the elfin_ethercat_driver/config/write_esi.yaml file and configure the parameters.

Then fix ESI with following commands.
```sh
sudo chrt 10 bash
roslaunch elfin_ethercat_driver elfin_esi_write.launch
```

Finally, restart the robot. Now ESI should be fixed.

Besides, some backup files of the original ESI of the slaves that are fixed are generated in elfin_ethercat_driver/script/ folder.
