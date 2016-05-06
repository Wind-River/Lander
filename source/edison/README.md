#Edison Board Set up

 * To set up the board
```
 configure_edison --setup
 ```
 * Copy Contents of this edision directory to /home/root on the edison board
 * Copy startup script to /etc/init.d/rocketLanderStart
 * Put rocketLanderStart in startup with 
 ```
update-rc.d -f rocketLanderStart defaults
```
 * Setup Complete
 
##Wifi
 * Will need to be reconfigured, atm only works at WIND 
 * configure_edison --wifi

##Bluetooth
 * Will show up as RocketLander in bluetooth list
 * Default bluetooth pin is 0505
 * Set up to start up automatically

##Arduino to Edison Serial
 * Set up to start up automatically
 * log file at /home/root/arduino_edison.log
 * User DEBUG  flag to send mock data back and forth. MEAN.io rest api call is not called.
```
 python arduino_edison.py DEBUG
 ```
 * Submit High Score Message and Reply
```
a2e:name=______,score=______;
e2a:ack;
```
 * Get HighScore Message and Reply
```
a2e:highscore?;
e2a:name=_____,score=______;
```

##Rest Api Calls
 * rocketLanderRest.py calls mean stack rest api. If host changes, change ipaddress here
 * a no args equals a get for high scorre, the first 5 is returned
 * pass a json string for submit. primary key is email address
```
    [{
        "email": "mojo3@jupiter.com",
        "lander_score": 999,
        "name": "Mojo Doggie 3",
        "phone": "555-555-5555",
        "reference": "510-749-2000"
    }]
```


