#!/bin/bash
sudo brctl addbr br0
sudo brctl addif br0 eth0
sudo brctl addif br0 eth1
sudo ifconfig br0 192.168.1.9 netmask 255.255.255.0 up
