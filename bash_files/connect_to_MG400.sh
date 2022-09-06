#!/bin/bash

sleep 10
nmcli con down 'Wired connection 1'

/etc/init.d/networking restart
ifconfig eth0 192.168.1.11

