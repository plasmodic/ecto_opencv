#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc, line_mod, objcog_db, tod

debug = True

plasm = ecto.Plasm()  #Constructor for Plasm

bin_color = line_mod.ColorMod(gsize=5,gsig = 2.0)
db_color = line_mod.ColorDebug();
coded_color = highgui.imshow(name="coded_color", waitKey=-1, autoSize=True)
raw_image = highgui.imshow(name="raw image", waitKey= -1, autoSize=True)
highgui_db_color = highgui.imshow(name="db_color", waitKey= -1, autoSize=True)

image_view = highgui.imshow(name="RGB", waitKey=0, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
db_reader = objcog_db.ObservationReader("db_reader", object_id="object_01")
templ_calc = line_mod.ColorTemplCalc()
print templ_calc.__doc__

plasm.connect(db_reader, "image", image_view, "input")
plasm.connect(db_reader, "mask", mask_view, "input")
plasm.connect(db_reader, "depth", depth_view, "input")

plasm.connect(db_reader, "image", bin_color, "image")
plasm.connect(db_reader, "image", raw_image, "input")
plasm.connect(bin_color, "output", coded_color, "input")
plasm.connect(bin_color, "output", db_color, "input")

plasm.connect(bin_color, "output", templ_calc, "image")
plasm.connect(db_reader, "mask", templ_calc, "mask")

plasm.connect(db_color, "output", highgui_db_color, "input")

if debug:
    ecto.view_plasm(plasm)

while(image_view.outputs.out != 27):
    try:
        #plasm will return non zero on exit conditions...
        if(plasm.execute(1)):
            break
    except Exception, e:
        print e
    

