#!/bin/bash
echo "Start camera stream"

sudo mkdir -p /tmp/stream
sudo chmod 777 /tmp/stream
#raspistill -w 1280 -h 720 -q 50 -ex backlight -mm backlit -o /tmp/stream/pic.jpg -tl 100 -t 9999999 -th 0:0:0 &
raspistill -w 2592 -h 1944 -q 50 -ex backlight -mm backlit -o /tmp/stream/pic.jpg -tl 100 -t 2147483647 -th 0:0:0 &
LD_LIBRARY_PATH=/usr/local/lib mjpg_streamer -i "input_file.so -f /tmp/stream -n pic.jpg" -o "output_http.so -w /usr/local/www"
