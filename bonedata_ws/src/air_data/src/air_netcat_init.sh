#!/bin/bash

rm /tmp/fifo
mkfifo /tmp/fifo
netcat -l -k -u -p 5001 < /tmp/fifo | netcat localhost 4002 > /tmp/fifo
