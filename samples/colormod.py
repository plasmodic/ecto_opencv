#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc, line_mod

debug = True

plasm = ecto.Plasm()  #Constructor for Plasm
video = highgui.VideoCapture(video_device=1)
bin_color = line_mod.ColorMod(thresh_bw = 20)
db_color = line_mod.ColorDebug();
coded_color = highgui.imshow(name="coded_color", waitKey=10, autoSize=True)
raw_image = highgui.imshow(name="raw image", waitKey=-1, autoSize=True)
highgui_db_color = highgui.imshow(name="db_color", waitKey=-1, autoSize=True)


if debug:
    ecto.print_module_doc(video)
    ecto.print_module_doc(coded_color)

plasm.connect(video, "out", bin_color, "image")
plasm.connect(video, "out", raw_image, "input")
plasm.connect(bin_color,"output",coded_color,"input")
plasm.connect(bin_color,"output",db_color,"input")
plasm.connect(db_color,"output",highgui_db_color,"input")

if debug:
    ecto.view_plasm(plasm)

thresh_gt = 10
bin_color.params.thresh_gt = thresh_gt;

bin_color.configure()
counter  = 0;
while(coded_color.outputs.out != 27):
    try:
#        counter += 1
#        if counter % 10 == 0:
#            #every 10 frames increment the threshhold
#            thresh_gt += 1
#            if thresh_gt > 250:
#                thresh_gt = 0;
#            bin_color.params.thresh_gt = thresh_gt;
#            bin_color.configure()
#            print "Trying threshold gt",thresh_gt

        plasm.execute()
    except Exception, e:
        print e
    

