# ROS_Assignment


Explaining the basics of what we have changed in james's code

1. To get parameters in the arguments x,y to get pick and place points
2. We have split these 2 points into 6 waypoints , having z_hover.
3. To work with gripper we have introduced open and close positions because it is a static information on which of these 6 waypoints we need to open it or close it.
4. To overcome -4 error that waypoints are being sent very fast we introduced motion.done flag that helps us to see if previous motion is completed or not.
5. To introduce pitch and yaw we have used math package of python(ref 1. https://bukkit.org/threads/tutorial-how-to-calculate-vectors.138849/, ref 2. https://uk.mathworks.com/matlabcentral/answers/298940-how-to-calculate-roll-pitch-and-yaw-from-xyz-coordinates-of-3-planar-points)
6. To work with arm and gripper introduced a flag for each of the node to be activated.
