#
# $Id: $
#
# Author(s):  Anton Deguet
# Created on: 2010-01-20
#
# (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
# 
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
# 
# --- end cisst license ---
#
# File based on FindNUMARRAY distributed with ITK 3.4 (see itk.org)
#
# Main modifications:
# - use Numpy instead of Numarray for all naming
# - added path for Python 2.5 and 2.6
# - renamed python script generated (det_npp became determineNumpyPath)
# - use lower case for CMake commands and keywords
# - updated python script to use get_include, not get_numpy_include which is now deprecated
#
# ---
#
# Try to find numpy python package
# Once done this will define
#
# PYTHON_NUMPY_FOUND        - system has numpy development package and it should be used
# PYTHON_NUMPY_INCLUDE_DIR  - directory(-ies) that should be include to use numpy headers
#
#
if(PYTHON_EXECUTABLE)
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/determineNumpyPath.py "try: import numpy; print numpy.get_include()\nexcept: pass\n")
    exec_program("${PYTHON_EXECUTABLE}"
                 ARGS "\"${CMAKE_CURRENT_BINARY_DIR}/determineNumpyPath.py\""
                 OUTPUT_VARIABLE PYTHON_NUMPY_INCLUDE_DIR
                 )
endif(PYTHON_EXECUTABLE)

if(PYTHON_NUMPY_INCLUDE_DIR)
    set(PYTHON_NUMPY_FOUND 1 CACHE INTERNAL "Python numpy development package is available")
endif(PYTHON_NUMPY_INCLUDE_DIR)
