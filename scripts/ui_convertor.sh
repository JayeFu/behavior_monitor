#!/bin/bash

default_src="../resources/TempWindow.ui"
default_dst="../src/behavior_monitor/TempWindow.py"

echo "Default src is ${default_src}"
choice="whatever"
while [[ ${choice} != "y" && ${choice} != "n" ]]
do
    read -p "Need to Change?[y/n]: " choice
    if test ${choice} == "y"
    then
        read -p "Please specify the src >> " src
    elif test ${choice} == "n"
    then
        src=${default_src}
    else
        echo "Only y/n accepted"
    fi
done

echo "src is set to ${src}"

echo "Default dst is ${default_dst}"
choice="whatever"
while [[ ${choice} != "y" && ${choice} != "n" ]]
do
    read -p "Need to Change?[y/n]: " choice
    if test ${choice} == "y"
    then
        read -p "Please specify the dst >> " dst
    elif test ${choice} == "n"
    then
        dst=${default_dst}
    else
        echo "Only y/n accepted"
    fi
done

echo "dst is set to ${dst}"

echo "Start Converting"
pyuic5 -x ${src} -o ${dst}
echo "Success"
