#!/bin/bash

ssh -i $(find -name *YonahAWSPrivateKey-2Jul19) ubuntu@ec2-18-138-24-228.ap-southeast-1.compute.amazonaws.com -L 4000:localhost:4000 -L 4001:localhost:4001 -L 4002:localhost:4002
