#!/usr/bin/env python

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

from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005 #0.005

    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.75
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    
    pcl_passthrough_pub.publish(pcl_to_ros(cloud_filtered))

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.025
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    inliers, coeficcients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative = False)
    extracted_outliers = cloud_filtered.extract(inliers, negative = True)
    cloud_table = extracted_inliers
    cloud_objects = extracted_outliers

    #publish to ros

    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(cloud_table))


    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.04)
    ec.set_MinClusterSize(80)
    ec.set_MaxClusterSize(3500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Publish ROS messages
    detected_objects_labels = []
    detected_objects = []

# Exercise-3 TODOs: 
    print("clusters detected: %s", np.array(cluster_indices).shape)
    for index, pts_list in enumerate(cluster_indices):
        print("processing cluster %s" %index)

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv = True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)

        feature = np.concatenate((chists, nhists))


        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)


        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cluster
        detected_objects.append(do)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects {}'.format(len(detected_objects_labels), detected_objects_labels))

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=10)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    pcl_passthrough_pub = rospy.Publisher("/pcl_passthrough", PointCloud2, queue_size=1)
    # TODO: Load Model From disk
    model_filename = 'model.sav'
    model = pickle.load(open(model_filename, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
