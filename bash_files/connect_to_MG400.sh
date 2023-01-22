#!/bin/bash

sleep 2
nmcli con down 'Wired connection 1'

#/etc/init.d/networking restart
echo '123'| sudo -S ifconfig eth0 192.168.1.11

