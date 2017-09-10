#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Statistical noise removal filter
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(10)  # Set the number of neighboring points to analyze for any given point
    x = 0.3  # Set threshold scale factor
    outlier_filter.set_std_dev_mul_thresh(x)  # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cloud_filtered = outlier_filter.filter()  # Finally call the filter function for magic

    # Voxel Grid filter downsampling
    vox = cloud_filtered.make_voxel_grid_filter()

    LEAF_SIZE = 0.01

    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough filter
    # Z-axis
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # Y-axis
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max = 0.4
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    # RANSAC plane segmentation
    seg = cloud_filtered.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(5000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])
                                             ])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    # Classify the clusters!
    detected_objects_labels = []
    detected_objects_list = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

        print('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    yaml_entries = []
    movement_plan = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    # Parse parameters into individual variables

    # Get the PointCloud for a given object and obtain its centroid
    labels = []
    centroids = []  # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # Rotate PR2 in place to capture side tables for the collision map
    # TODO: doesn't do anything
    # pr2_joint_pub.publish(-np.pi / 2)  # look to the left
    # pr2_joint_pub.publish(np.pi)       # look to the right
    # pr2_joint_pub.publish(-np.pi / 2)  # center

    # Loop through the pick list
    for i in range(len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        print("Required to pick up", object_name,"in ",object_group)

        # find desired object amongs recognized objects if possible
        idx = None
        for j in range(len(object_list)):
            if labels[j] == object_name:
                idx = i
                break

        if idx is not None:
            print("Found",object_name,"at index",idx)
            object = object_list[idx]
            centroid = centroids[idx]

            # Create 'place_pose' for the object
            # pick position should be where object's centroid is
            pick_location = Pose()
            pick_location.position.x = float(centroid[0])
            pick_location.position.y = float(centroid[1])
            pick_location.position.z = float(centroid[2])

            # Assign the arm to be used for pick_place
            # choose arm for object, green is on the right, red is on the left
            arm = String()
            arm.data = "right"
            if object_group == "red":
                arm.data = "left"

            dropbox = None
            for dropbox_info in dropbox_list_param:
                if dropbox_info["group"] == object_group:
                    dropbox = dropbox_info
                    break

            drop_location = dropbox["position"]
            drop_location[0] += (np.random.random() - 0.5) * 0.07  # 7 cm random x offset
            drop_location[1] += (np.random.random() - 0.5) * 0.07  # 7 cm random y offset

            place_location = Pose()
            place_location.position.x = drop_location[0]
            place_location.position.y = drop_location[1]
            place_location.position.z = drop_location[2]

            # other parameters
            world_number = Int32()
            #world_number.data = rospy.get_param("/world_name")   # Hmm, KeyError on this one
            # world_number.data = 1
            # world_number.data = 2
            world_number.data = 3
            pick_name = String()
            pick_name.data = object_name

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            entry = make_yaml_dict(world_number, arm, pick_name, pick_location, place_location)
            yaml_entries.append(entry)

            #'''
            # PR2 simulation is really flaky, seems like a lot of work needs to be done to make pick reliable :-(
            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')
            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # Insert your message variables to be sent as a service request
                resp = pick_place_routine(world_number, pick_name, arm, pick_location, place_location)

                print ("Response: ", resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            #'''

    # Output your request parameters into output yaml file
    send_to_yaml("output_" + str(world_number.data) + ".yaml", yaml_entries)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", pc2.PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", pc2.PointCloud2, queue_size=1)
    pr2_joint_pub = rospy.Publisher("/pr2/join_controller/command", Float64, queue_size=3)

    # Load Model From disk
    # model = pickle.load(open('model_world1.sav', 'rb'))  # World 1
    # model = pickle.load(open('model_world2.sav', 'rb'))  # World 2
    model = pickle.load(open('model_world3.sav', 'rb'))  # World 3
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
