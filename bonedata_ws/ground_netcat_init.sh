rm /tmp/fifo
mkfifo /tmp/fifo
netcat -l -k -p 4002 < /tmp/fifo | netcat -u localhost 5001 > /tmp/fifo
