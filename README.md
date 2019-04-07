# vehicle-velocity-estimator
This projects aims to estimate the velocity of vehicles moving on road from the CCTV footage of the road.

## Requirements
* OpenCV2
> sudo apt-get install python-opencv

## Instructions to run the code
* Inside `global_variables.py` replace `video_path` with the path of your video
* Replace `top_left` and `bottom_right` with the coordinates of the box inside which you want to track the object
* Run the file named `automatic_track.py` using `python2` or simply `python automatic_track.py`
* Use `ESC` key to exit from the video
