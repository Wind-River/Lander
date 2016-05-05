#!/bin/sh
# need the sleeps so that it has time to initialize everything
/bin/date > /home/root/bluetoothstartup.log
echo "bluetoothstartup.sh starting..." >> /home/root/bluetoothstartup.log
/usr/sbin/rfkill unblock bluetooth
sleep 3
/usr/bin/hciconfig hci0 up
sleep 3
/usr/bin/hciconfig hci0 piscan
sleep 3
/usr/bin/hciconfig hci0 sspmode 0
sleep 3
/home/root/simple-agent.py &
/usr/bin/hciconfig hci0 sspmode 0
sleep 3
echo "bluetoothstartup.sh finished" >> /home/root/bluetoothstartup.log
