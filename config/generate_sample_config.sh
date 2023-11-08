#!/bin/bash
cat config.actual.ini | sed -e "s/wifi=.*/wifi=ssid:password/" | sed -e "s/mqtt_broker=.*/mqtt_broker=brokerhostname.local/" | sed -e "s/mqtt_username=.*/mqtt_username=username/" | sed -e "s/mqtt_password=.*/mqtt_password=password/" > config.ini
cat config.ini
