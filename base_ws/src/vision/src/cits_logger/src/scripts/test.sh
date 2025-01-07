#!/bin/bash


cat $1/logs/logback.2024-10-12.0.log | tail -1000000 | python3 $1/log_maker.py 