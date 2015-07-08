#!/bin/bash

echo $1

echo "start_managed.sh" | nc 192.168.2.2 8888 -q 2
