#!/usr/bin/env python
import ecto
from ecto_opencv import highgui
from ecto_opencv import imgproc

#from ecto_opencv import opencv

debug = False

class SobelAbs(ecto.Module):
    def __init__(self, **kwargs): 
        #call init, which will thunk to Params, then to config
        ecto.Module.__init__(self,**kwargs)
        
    @staticmethod
    def Params(params):
        #declare parameters, with defualt values
        params.declare("x", "gradient x", 0)
        params.declare("y", "gradient y", 0)

    def config(self):
        #declare inputs/outputs, these are exposed to outside world
        self.inputs.declare("input","an image", None)
        self.outputs.declare("gradient", "a gradient image", None)
        self.outputs.declare("out", "an absolute gradient image", None)
        
        #setup internal plasm here:
        self.plasm = ecto.Plasm()
        self.sobel = imgproc.Sobel(x = self.params.x, y = self.params.y)
        self.abs = imgproc.AbsNormalized()
        
        self.sobel.inputs["input"].connect(self.inputs["input"])
        self.outputs["out"].connect(self.abs.outputs["out"])
        self.outputs["gradient"].connect(self.sobel.outputs["out"])
        
        self.plasm.connect(self.sobel,"out",self.abs,"input")   

        if debug:
            ecto.view_plasm(self.plasm)

    def process(self):
        #local vars
        plasm = self.plasm
        sobel = self.sobel
        abs = self.abs
                
        plasm.mark_dirty(sobel) #mark the sobel dirty
        plasm.go(abs)

        
        
video = highgui.VideoCapture()
imshow = highgui.imshow(name="video",waitKey=2)
sobelShower = highgui.imshow(name="sobel",waitKey=-1)
grayShower = highgui.imshow(name="gray",waitKey=-1)
gradientShow = highgui.imshow(name="gradient",waitKey=-1)

sobelX = SobelAbs( x= 1, y = 0)
sobelY = SobelAbs( x= 0, y = 1)

rgb2gray = imgproc.cvtColor(flag=7)

adder = imgproc.ImageAdder()

plasm = ecto.Plasm()
plasm.connect(video, "out", imshow, "input")
plasm.connect(video, "out", rgb2gray , "input")
plasm.connect(rgb2gray, "out", sobelX, "input")
plasm.connect(rgb2gray, "out", sobelY, "input")
plasm.connect(sobelX, "out", adder , "a")
plasm.connect(sobelY, "out", adder , "b")
plasm.connect(rgb2gray, "out", grayShower,"input")
plasm.connect(adder, "out", sobelShower, "input")
plasm.connect(sobelX, "gradient", gradientShow, "input")

#print plasm.viz()
ecto.view_plasm(plasm)

while(imshow.outputs.out != 27):
    plasm.mark_dirty(video)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(sobelShower)
    plasm.go(grayShower)
    plasm.go(imshow)
    plasm.go(gradientShow)


#def test_python_module():
#    mod = Identity(factor = 5.3)
#    mod.inputs.input = 10
#    mod.process()
#    print mod.outputs.out
#    assert mod.outputs.out == 10
#
#if __name__ == '__main__':
#    test_python_module()
