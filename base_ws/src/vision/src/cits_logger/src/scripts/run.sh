#!/bin/bash

java -jar $1/cits/trafficTotalFromRSUConsole.jar | python3 $1/log_maker.py 


# cat $1/logs/10201215.txt | python3 $1/log_maker.py 
