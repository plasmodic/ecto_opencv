#!/bin/sh

DIFFCMD="colordiff -w -y -W 155"

doit () { 
    clear
    $DIFFCMD $1 $2 | less -R #press q in less to quit.
    python $2
    echo $1 $2
}

first () {
  clear
  cat example_01.py | less -R #press q in less to quit.
  python example_01.py
}

first
doit example_01.py example_02.py
doit example_02.py example_03.py
doit example_03.py example_06.py
doit example_06.py example_07.py
doit example_07.py example_08.py
