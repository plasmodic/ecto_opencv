#!/bin/sh -e
$1 --scheduler Singlethreaded --niter 100
$1 --scheduler Threadpool --niter 100
