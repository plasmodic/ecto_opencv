#!/bin/python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui,calib,imgproc, cv_bp as opencv

debug = True

def calibration(rows,cols,square_size,pattern_type,n_obs,video):
    plasm = ecto.Plasm()
    pattern_show = highgui.imshow(name="pattern", waitKey=10, autoSize=True)
    rgb2gray = imgproc.cvtColor(flag=7)
    circle_detector = calib.PatternDetector(rows=rows, cols=cols,pattern_type=pattern_type,square_size=square_size )
    ecto.print_module_doc(circle_detector)
    circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
    camera_calibrator = calib.CameraCalibrator(output_file_name="camera.yml",n_obs=n_obs)
    
    plasm.connect(video, "image", rgb2gray, "input")
    plasm.connect(rgb2gray, "out", circle_detector, "input")
    plasm.connect(video, "image", circle_drawer, "input")
    plasm.connect(video, "image", camera_calibrator, "image")
    plasm.connect(circle_detector, "out", circle_drawer, "points")
    plasm.connect(circle_detector, "found", circle_drawer, "found")
    plasm.connect(circle_drawer, "out", pattern_show, "input")
    plasm.connect(circle_detector, "ideal", camera_calibrator,"ideal")
    plasm.connect(circle_detector, "out", camera_calibrator,"points")
    plasm.connect(circle_detector, "found", camera_calibrator, "found")
    
    print plasm.viz()
    ecto.view_plasm(plasm)
    
    while(pattern_show.outputs.out != 27 and camera_calibrator.outputs.calibrated == False):
        plasm.execute(1)

if __name__ == "__main__":
    use_kinect = True

    video = None
    if not use_kinect :
        video = highgui.VideoCapture(video_device=0)
    else:
        video = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_VGA_30HZ)
        
    calibration(rows=7,cols=3,square_size=0.03,pattern_type="acircles",n_obs=50,video=video)