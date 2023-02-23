#!/bin/bash

clear
mkdir build
cd build
cmake ..
make -j8 -B

# Fill in the various paths involved in the conversion

calib_file=/home/antoine/Documents/data/Sequence3/calibration/calib_twin_fisheye.xml
image_dir=/home/antoine/Documents/data/Sequence4/images/
mask=/home/antoine/Documents/data/Sequence3/calibration/maskFull.png
poses_fic=/home/antoine/Documents/data/Sequence4/poses.txt

echo "variables initialized"    

./dual2dual $calib_file $image_dir 0 20 1 $mask $poses_fic 1