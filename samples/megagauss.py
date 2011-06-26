#!/usr/bin/env python
import ecto
from ecto_opencv import imgproc, highgui
import time, signal, sys

debug = True
k = 16

video = highgui.VideoCapture(video_device=0)
g = imgproc.GaussianBlur(sigma=5)
graph = []

graph.append(video['image'] >> g['input'])

for x in range(k):
    nextg = imgproc.GaussianBlur(sigma=5)
    graph.append(g['out'] >> nextg['input'])
    g = nextg

fps = highgui.FPSDrawer()
graph += [
          g['out'] >> fps[:],
          fps[:] >> highgui.imshow("megagauss", name="megagauss", waitKey=1)[:]
          ]

plasm = ecto.Plasm()
plasm.connect(graph)
if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

if __name__ == "__main__":
    nthreads = 1
    if len(sys.argv) > 1 :
        nthreads = int(sys.argv[1])

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(nthreads)

