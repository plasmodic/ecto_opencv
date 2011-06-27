#!/usr/bin/env python
import ecto
from ecto_opencv import imgproc, highgui
import time, signal, sys


def make_graph(k,sigma, debug):

    video = highgui.VideoCapture(video_device=0)
    g = imgproc.GaussianBlur(sigma=sigma)
    graph = []
    graph += [video['image'] >> g['input']]
    for x in range(k):
        nextg = imgproc.GaussianBlur(sigma=sigma)
        graph += [g['out'] >> nextg['input']]
        g = nextg
    fps = highgui.FPSDrawer()
    graph += [
              g['out'] >> fps[:],
              fps[:] >> highgui.imshow("megagauss", name="megagauss", waitKey=10)[:]
              ]
    return graph

if __name__ == "__main__":
    nthreads = 1
    k = 4
    sigma = 2
    debug = True
    if len(sys.argv) not in [4,1] :
        print "usage :",sys.argv[0], "nthreads","sigma","K"
        sys.exit(-1)
    if len(sys.argv) == 4:
        nthreads = int(sys.argv[1])
        sigma = int(sys.argv[2])
        k = int(sys.argv[3])
    plasm = ecto.Plasm()
    plasm.connect(make_graph(k,sigma,debug))
    if debug:
        print plasm.viz()
        ecto.view_plasm(plasm)
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(nthreads)
    #TODO seperate call to print stats.

