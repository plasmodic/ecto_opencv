#!/bin/sh

DIFFCMD="colordiff -w -y -W 155"

doit () { 
    clear
    $DIFFCMD $1 $2
    read NEXT
}

doit example_01.py example_02.py
doit example_02.py example_03.py

     