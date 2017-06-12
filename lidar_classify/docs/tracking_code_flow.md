# Tracking code flow

## Target Class
Store the all the information of the target. 
Especially, the vanish time count for vanish target to determine the target is disappear or not.

## TargetKalman Class
Every tracked serial targets store in the circular_buffer named target_buffer_. And every tracked serial targets only hold one kalman.

New tracked serial targets use another TargetKalman Class.

## Tracking Class
The main class for tracking.

### Input message
convertMsgForTracking converts the lidar::msg::clusterArray to segmentation message which contains the cuboid information and the classification label and the pointcloud of the segmentations.

### Tracking main function
* initForTracking 
Initializes the segmentation message to the target information.

* readSelfVelocity
Read the self velocity from the offline velocity file.

* calculateTransformation
Calculate the transformation matrix based on the self velocity, not used now.

* getRectInfo
Get the rectangle information include, include the main direction of the target.

* trackTarget
Track the target, the very important function. 

Association:
1. The first association
Based on the length width ratio, pointcloud number, distance, size between the current target and last target, if match the threshold, do the kalman update.

2. The second association
Based only the distance between the current and last target to slove the target with longer distance can't be tracked problem.


Update the tracked target information using kalman:

When association match, do the kalman update.

* trackVanishTarget
Track the vanish target, if the the disappear times > TRACKING_VANISH_TIME_, consider the target is disappeared.

* smoothFiterForVel
Filter the velocity for the target using smooth filter, mainly for better velocity.

* trackLowVelTarget
Track low velocity target, for now only filter the velocity.
Why use the function accoring to the smoothFilterForVel, to be consider?

* filterDirection
Filter the direction, to get robust direction.

* addCurrentTarget
Add the new targets to the tracks_vector.

* releaseTarget
Release the resource, current empty funtion.
