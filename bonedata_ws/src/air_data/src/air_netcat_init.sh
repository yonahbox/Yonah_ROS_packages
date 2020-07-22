#!/bin/bash

rm /tmp/fifo
mkfifo /tmp/fifo
netcat -l -u -v -p 5001 < /tmp/fifo | netcat -v localhost 4002 > /tmp/fifo
