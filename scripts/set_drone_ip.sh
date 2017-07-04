#!/bin/bash
DRONE_ESSID=$(iwconfig | grep -o 'ardrone2_[0-9]*')
echo 'Connected to' $DRONE_ESSID
DRONE_IP=$(grep $DRONE_ESSID drone_ips.txt | grep -o '192.168.10.[0-9]*')
echo 'Setting IP to' $DRONE_IP
echo "iwconfig ath0 mode managed essid dronehub ; ifconfig ath0" $DRONE_IP "netmask 255.255.255.0 up ; route add default gw 192.168.10.10"
echo "iwconfig ath0 mode managed essid dronehub ; ifconfig ath0" $DRONE_IP "netmask 255.255.255.0 up ; route add default gw 192.168.10.10" | nc -w 5 192.168.1.1 23
ping -c 10 $DRONE_IP
