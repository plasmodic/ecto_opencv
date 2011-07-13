#!/usr/bin/env python
# Simple vid cap
import ecto
from ecto_opencv import highgui, calib, imgproc, projector

plasm = ecto.Plasm()
sched = ecto.schedulers.Singlethreaded(plasm)

pattern_draw = projector.PatternProjector()

video_display = highgui.imshow('pattern',
                               name='video_cap', waitKey=2, maximize=True)

plasm.connect(pattern_draw['pattern'] >> video_display['input'],
              )

ecto.view_plasm(plasm)
sched.execute()
