rm /tmp/fifo
mkfifo /tmp/fifo
netcat -l -k -p 4000 < /tmp/fifo | netcat -u localhost 5001 > /tmp/fifo
