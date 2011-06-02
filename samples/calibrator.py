#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc

debug = True

def calibration(rows,cols,square_size,pattern_type):
    plasm = ecto.Plasm()
    
    pattern_show = highgui.imshow(name="pattern", waitKey=10, autoSize=True)
    rgb2gray = imgproc.cvtColor(flag=7)
    video = highgui.VideoCapture(video_device=0)
    circle_detector = calib.PatternDetector(rows=rows, cols=cols,pattern_type=pattern_type,square_size=square_size )
    ecto.print_module_doc(circle_detector)
    circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
    camera_calibrator = calib.CameraCalibrator(output_file_name="camera.yml",n_obs=25)
    
    plasm.connect(video, "out", rgb2gray, "input")
    plasm.connect(rgb2gray, "out", circle_detector, "input")
    plasm.connect(video, "out", circle_drawer, "input")
    plasm.connect(video, "out", camera_calibrator, "image")
    plasm.connect(circle_detector, "out", circle_drawer, "points")
    plasm.connect(circle_detector, "found", circle_drawer, "found")
    plasm.connect(circle_drawer, "out", pattern_show, "input")
    plasm.connect(circle_detector, "ideal", camera_calibrator,"ideal")
    plasm.connect(circle_detector, "out", camera_calibrator,"points")
    plasm.connect(circle_detector, "found", camera_calibrator, "found")
    
    print plasm.viz()
    ecto.view_plasm(plasm)
    
    while(pattern_show.outputs.out != 27 and camera_calibrator.outputs.calibrated == False):
        plasm.execute()

if __name__ == "__main__":
    calibration(rows=7,cols=3,square_size=0.03,pattern_type="acircles")