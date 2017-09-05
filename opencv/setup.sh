#!/bin/bash

cmake -DCMAKE_BUILD_TYPE=RELEASE \
 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules \
 -DWITH_TBB=ON \
 -DBUILD_NEW_PYTHON_SUPPORT=ON \
 -DINSTALL_C_EXAMPLES=ON \
 -DINSTALL_PYTHON_EXAMPLES=ON \
 -DBUILD_EXAMPLES=ON \
 -DWITH_CUDA=OFF \
 -DBUILD_TIFF=ON \
 -DCMAKE_INSTALL_PREFIX=$(python2.7 -c "import sys; print(sys.prefix)") \
 -DPYTHON_EXECUTABLE=(which python2.7) \
 -DPYTHON_INCLUDE_DIR=$(python2.7 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
 -DPYTHON_PACKAGES_PATH=$(python2.7 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
 ../opencv-3.3.0
