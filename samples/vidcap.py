#!/usr/bin/env python
#
# Simple vid cap
#
import ecto, ecto.opts
from ecto_opencv import highgui, calib, imgproc

video_cap = highgui.VideoCapture(video_device=0, width=640, height=480)

fps = highgui.FPSDrawer()

video_display = highgui.imshow('imshow',
                               name='video_cap', waitKey=2)

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> video_display['input'],
              )

if __name__ == '__main__':
    ecto.opts.parse_args_run_plasm(plasm,
                                   description='Capture a video from the device and display it.'
                                   )
