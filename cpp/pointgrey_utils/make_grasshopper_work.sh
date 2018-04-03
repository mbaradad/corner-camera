#!/bin/bash
#This seems to for make grasshopper work
#model Point Grey Research Grasshopper3 GS3-U3-23S6C 14110929
#the Monochrome does not need this

sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
