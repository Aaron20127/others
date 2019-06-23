#!/bin/bash

### usage: sh catalog.sh test.py
### Print out all the python functions and sort them

if [ "$#" = "1" ]; then
    var_name=$(basename ${1})
    grep -n "^def .*(.*):$" ${1} | \
        sed "s/^/${var_name} /" | \
        sed 's/:def / /g' | \
        sed 's/(.*://' | \
        awk '{print $3" "$1":"$2}' | sort
else
    grep -n "^def .*(.*):$" ${@} | \
        sed 's/^.*\///g' | \
        sed 's/:def / /g' | \
        sed 's/(.*://' | \
        awk '{print $2" "$1}' | sort
fi

