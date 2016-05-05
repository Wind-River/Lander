#Edison Board Set up

 * Startup script is /etc/init.d/rocketLanderStart
 * All scripts have been copied to /home/root on the edison board
 * u:p root:rocketLander

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
 * Pass DEBUG to send mock data back and forth by passing rest api call. e.g. python arduino_edison.py DEBUG

##Rest Api Calls
 * rocketLanderRest.py calls mean stack rest api. If host changes, change ipaddress here
 * a no args equals a get for high scorre, the first 5 is returned
 * pass a json string for submit. primary key is email address
```
    [{
        "email": "mojo3@jupiter.com",
        "lander_score": 999,
        "name": "Mojo Doggie 3"
        "phone": "555-555-5555"
        "reference" "510-749-2000"
    }]
```


