#!/bin/sh

#
#  Copyright (c) 2016 Wind River Systems, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#

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
