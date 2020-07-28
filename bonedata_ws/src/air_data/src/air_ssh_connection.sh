#!/bin/bash

ssh $1@$2 -L 4000:localhost:$3 -L 4001:localhost:$4 -L 4002:localhost:$5
