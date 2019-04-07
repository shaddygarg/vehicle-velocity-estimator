# vehicle-velocity-estimator
This projects aims to estimate the velocity of vehicles moving on road from the CCTV footage of the road. 

The implementation uses Lucas Kanade algorithm along with Optical Flow for generating a set of good points and tracking then for estimating the velocity of the moving object. Background removal using KNN algorithm is used for instantiating the tracking process whenever a new object is detected.

## Requirements
* OpenCV2
> sudo apt-get install python-opencv
* Python2.x
* numpy
> sudo pip install numpy

## Instructions to run the code
* Inside `global_variables.py` replace `video_path` with the path of your video
* Replace `top_left` and `bottom_right` with the coordinates of the box inside which you want to track the object
* Run the file named `automatic_track.py` using `python2` or simply `python automatic_track.py`
* Use `ESC` key to exit from the video
