# Udacity RoboND Perception Exercise

ROS Script. Segments objects out of a rgbd camera image and recognizes them. 

[screenshot]: ./screenshot.png
![screenshot]
## Methods used

+ Passthrough filtering
+ RANSAC Plane Segmentation
+ Euclidean Clusteruing
+ SVM Algorithm to seperate objects (color histograms and normal histograms)