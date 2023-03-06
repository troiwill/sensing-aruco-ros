cmake_minimum_required(VERSION 3.0.0)
project(sense_aruco_msgs)

find_package(catkin REQUIRED
    message_generation
    geometry_msgs
    std_msgs
)

add_message_files(
    FILES
    Marker.msg
    MarkerArray.msg
    MarkerPose.msg
    MarkerPoseArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)
