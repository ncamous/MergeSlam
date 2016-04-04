This is a catkin_ws must be initialized as such.

To test the code:

Launch the lsd_merge.launch
Rosbag play the bag files form the 2 lsd_slams containing the keyframe and graph messages.
You can download them here:

https://www.dropbox.com/sh/cks6418g0vku3aa/AAAIHgK4ULx014ZGLY87HKkja?dl=0



It is better to launch each bag with a time ga, if not you can just set the correct path to the bag files by editing the bag_lsd12.launch and will be launched simultaneously

For the "place recognizer" which launches the view-overlapping msg, launch the lsd_sim3track file.


