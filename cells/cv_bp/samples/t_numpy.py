import ecto_opencv.cv_bp as cv_bp
import numpy
c  = cv_bp.Mat()
npa =numpy.array([[2,0,0],[0,3,2],[2,2,3]],dtype=numpy.int32)
c.fromarray(npa)
print c
a = c.toarray()
print a
