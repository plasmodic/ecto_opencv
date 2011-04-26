#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv import imgproc

#from ecto_opencv import opencv

debug = False

class SobelAbs(ecto.Module):
    def __init__(self, **kwargs): 
        #call init, which will thunk to Params, then to config
        ecto.Module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        #declare parameters, with defualt values
        params.declare("x", "gradient x", 0)
        params.declare("y", "gradient y", 0)

    def config(self):
        #declare inputs/outputs, these are exposed to outside world
        self.inputs.declare("input", "an image", None)
        self.outputs.declare("gradient", "a gradient image", None)
        self.outputs.declare("out", "an absolute gradient image", None)
        
        #setup internal plasm here:
        self.plasm = ecto.Plasm()
        self.sobel = imgproc.Sobel(x=self.params.x, y=self.params.y)
        self.abs = imgproc.AbsNormalized()
        
        
        self.sobel.inputs["input"].connect(self.inputs["input"])
        self.outputs["out"].connect(self.abs.outputs["out"])
        self.outputs["gradient"].connect(self.sobel.outputs["out"])
                
        self.plasm.connect(self.sobel, "out", self.abs, "input")   

        if debug:
            ecto.view_plasm(self.plasm)

    def process(self):
        #local vars
        plasm = self.plasm
        sobel = self.sobel
        abs = self.abs
        plasm.mark_dirty(sobel) #mark the sobel dirty
        plasm.go(abs)


class SobelXY(ecto.Module):
    def __init__(self, **kwargs): 
        #call init, which will thunk to Params, then to config
        ecto.Module.__init__(self, **kwargs)
        
    @staticmethod
    def Params(params):
        pass

    def config(self):
        #declare inputs/outputs, these are exposed to outside world
        self.inputs.declare("input", "an image", None)
        self.outputs.declare("out", "an sobel XY image", None)
        self.outputs.declare("x", "an sobel X image", None)
        self.outputs.declare("y", "an sobel Y image", None)
        
        #setup internal plasm here:
        plasm = ecto.Plasm()
        sobelX = SobelAbs(x=1, y=0)
        sobelY = SobelAbs(x=0, y=1)
        adder = imgproc.ImageAdder()

        plasm.connect(sobelX, "out", adder , "a")
        plasm.connect(sobelY, "out", adder , "b")
     
        #tie the inputs to the outputs
        sobelX.inputs["input"].connect(self.inputs["input"])
        sobelY.inputs["input"].connect(self.inputs["input"])
        self.outputs["out"].connect(adder.outputs["out"])
        self.outputs["x"].connect(sobelX.outputs["out"])
        self.outputs["y"].connect(sobelY.outputs["out"])

        self.plasm = plasm
        self.input_modules = (sobelX, sobelY)
        self.output_modules = (adder,)
        
        if debug:
            ecto.view_plasm(self.plasm)

    def process(self):
        for input in self.input_modules:
            self.plasm.mark_dirty(input)
        for output in self.output_modules:
            self.plasm.go(output)


def video_test():
    video = highgui.VideoCapture()
    imshow = highgui.imshow(name="video", waitKey=2)
    sXY = SobelXY()
    sobelshow = highgui.imshow(name="sobel", waitKey= -1)

    plasm = ecto.Plasm()
    plasm.connect(video, "out", imshow, "input")
    plasm.connect(video, "out", sXY, "input")
    plasm.connect(sXY, "out", sobelshow, "input")
    
    if debug:
        ecto.view_plasm(plasm)

    while(imshow.outputs.out != 27):
        plasm.mark_dirty(video)
        # TODO just call go on the whole plasm, to trigger all leaves being called. 
        plasm.go(sobelshow)
        plasm.go(imshow)

if __name__ == '__main__':
    video_test()

