#!/bin/bash

echo "# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d" > /etc/network/interfaces.BK

echo "# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

auto eth0
iface eth0 inet static
   address 192.168.1.10
   netmask 255.255.255.0
   gateway 192.168.1.1
   dns_nameservers 192.168.1.1 8.8.8.8" > /etc/network/interfaces

/etc/init.d/networking restart
echo "press any key to finish this connection"
while [ true ] ; do
read -t 3 -n 1
if [ $? = 0 ] ; then
sudo rm -r /etc/network/interfaces
sudo mv /etc/network/interfaces.BK /etc/network/interfaces
/etc/init.d/networking restart
echo "please disconnect eth0"
exit ;
else
echo "waiting for the keypress"
fi 
done

