#!/bin/sh
echo "uploading program '$1' to the robot..."
scp $1 robot@192.168.1.110:~
