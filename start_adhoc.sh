#!/bin/bash

echo $1

echo "start_adhoc.sh" | nc $1 8888 -q 2
