#!/bin/bash

ssh -i /home/ubuntu/Yonah_ROS_packages/src/air_data/src/YonahAWSPrivateKey-30Jul19 ubuntu@ec2-18-138-24-228.ap-southeast-1.compute.amazonaws.com -L 4000:localhost:4000 -L 4001:localhost:4001 -L 4002:localhost:4002

# Remarks: Let's get the $find function to work, so that we don't have to keep specifying the absolute path!