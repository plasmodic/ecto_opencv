#!/bin/sh -e
cd $1
python -c "__import__('$2'.split('/')[-1].split('.py')[0])"


