# README #

### Package summary ###

The *drone_visual_odometry* package includes the visual odometry algorithm originally developed by Abdulla Al Kaff.


## ** Inputs (Subscribed Topics) ** ##
* *webcam/image* ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

    Current image.

* *webcam/camera_info* ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

    Camera parameters.

* *plane_distance* ([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))

    Distance to the plane.


## ** Outputs (Published Topics) ** ##
* *localization/visual_pose* ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

    Local pose of the vehicle.

## ** Parameters ** ##
* *~camera_topic* (string)

    Topic where the camera image and the camera_info are published.

* *~altitude_topic* (string)

    Topic where the distance to the plane is published.

* *~pose_topic* (string)

    Output topic.
