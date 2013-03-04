kinect_demos
============

** Overview **

This package is meant to host a few demos built around integrating advanced
perception capabilities through the Microsoft Kinect sensor and ROS for the 
Kinova Jaco arm.
As of 2011/12/20, only such a demo exists: a basic cylinder detection and
grasping procedure.

** Cylinder grasping **

This demo requires a Kinect sensor and a known fixed transform between the
sensor's base and the Jaco's base.
It will automatically detect cylinders in front of the sensor and generate a
3-point cartesian trajectory to grasp and lift the cylinder from its top.

*** Detection ***
The cylinder detection works by extracting the best-fitting cylinder in a 
given scene.
It assumes the cylinder lays on a regular plane.
For best results, the plane should completely fill the sensor's field-of-view, 
and the object meant to be grasped should be the only cylinder-like present in 
the scene.
The full detection pipeline in PCL is described in
launch/extract_cylinder.launch, but here are the principal steps:

Voxel generation --> Normals estimation --> RANSAC Plane fitting --> 
Plane extraction --> RANSAC cylinder model fitting --> 
Cylinder exclusion & projection --> Cylinder model

*** Trajectory generation ***
The cylinder's model is then consumed by the cylinder_to_pose node
(src/cylinder_to_pose.cpp). 
The center of gravity, height and radius are then used to generate poses for
the five-step trajectory and state machine:

 - IDLE: Waits for a new cylinder's pose.
 - OPEN_HAND: Open the Jaco's fingers.
 - GO_TOWARD: Go to a position hovering the cylinder.
 - LOWER: Go down to the top of the cylinder, the fingers will reach lower than
   than the cylinder's top.
 - CLOSE_HAND: Closes the Jaco's fingers.
 - LIFT: Go back to the GO_TOWARD pose hovering the cylinder's original
   position.

After LIFT, the state machine reverts back to IDLE, so the cylinder will get
dropped if a new cylinder's pose is given.

*** Operation ***

To run the demo, you first need to configure the static transform between the 
robot's base (jaco_base_link) and the Kinect's reference frame (camera_link).
This is done in the launch/jaco_kinect.launch file at line 11 using the standard
TF's static_transform_publishers' syntax (X Y Z yaw pitch roll parent_frame
child_frame publication_frequency).
To figure out the orientation of the frames and validate your transform, you can 
position Jaco's hand in front of the Kinect and visualize the point cloud with
rviz.
If the cloud matches the robot's model, your transform is correct.

To launch the Kinect, simply run the default OpenNI launch file: 
 - roslaunch openni_launch openni.launch
In some cases, the sensor isn't initialized properly.
To quickly check if it works correctly, you can use "rostopic bw ..." with any
topic in the /camera namespace. 
If the bandwith is anything but 0, the sensor is ready.
You can keep this launch file running in the background.

You then need to launch Jaco's node in standalone mode:

 - roslaunch jaco_node standalone.launch

The robot should go in retract/ready mode.
You can then start the cylinder extractor and pose generator:
 
 - roslaunch kinect_demos extract_cylinder.launch

The pose generator exposes three services to control the demo:

 - /arm_goal/seek, which resets cylinder extraction.
 - /arm_goal/go, which sends the trajectory through the "hand_goal" topic.

This node always publishes the 3 points in the planned trajectory in 3
different topics: 

 - /arm_goal/toward, hovering
 - /arm_goal/target, top of the cylinder
 - /arm_goal/lifted, hovering (same as toward)

You can check the planned pose in rviz to confirm it detected the right
object.
The cylinder's pose is stabilized with a rolling average, the /arm_goal/seek
resets this average.
After a few seconds, you should obtain a stabilized pose free of the noise
normally present in the cylinder's extraction process.
You can call the seek process easily through the shell:

 - rosservice call /arm_goal/seek

When the trajectory is satifactory, you can start the arm's movement:

 - rosservice call /arm_goal/go

If you restart the process by recalling both services, do not forget that the
previously grasped cylinder will be dropped in the OPEN_HAND state, and that
Jaco's forearm will probably detected as a cylinder anyway.
Considering this, the Jaco's arm should be retracted first by calling this
service:

 - rosservice call /jaco_arm/retract



