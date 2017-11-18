# Project: Perception Pick & Place
## Steps to complete the project
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Writeup / README

* 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
You're reading it!

[//]: # (Image References)
[statf]: ./misc_images/statf.png
[voxelf]: ./misc_images/voxelf.png
[passf]: ./misc_images/passf.png
[ransacf]: ./misc_images/ransacf.png
[clusters]: ./misc_images/clusters.png
[svm_model]: ./misc_images/svm_model.png
[world1]: ./misc_images/world1.png
[world2]: ./misc_images/world2.png
[world3]: ./misc_images/world3.png
[improvements]: ./misc_images/improvements.png
[improvements2]: ./misc_images/improvements2.png

## Exercise 1, 2 and 3 pipeline implemented
* Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Most of the parameters chosen for exercise 1 have been reused in this project. One of the changes was in the passthrough filter. Not only I filtered the `Z` axis but also on the `X` axis to filter out part of the boxes.

**Statistical Outlier Filtering**
```python
# Statistical Outlier Filtering
outlier_filter = cloud.make_statistical_outlier_filter()
outlier_filter.set_mean_k(25)
scale = 0.1
outlier_filter.set_std_dev_mul_thresh(scale)
cloud_filtered = outlier_filter.filter()
```

![Statistical filtering][statf]

**Voxel Grid Downsampling**
```python
# Voxel Grid Downsampling
vox = cloud_filtered.make_voxel_grid_filter()
LEAF_SIZE = 0.01
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
cloud_filtered = vox.filter()
```

![Voxel Grid Downsampling][voxelf]

**Passthrough Filter**
```python
# Passthrough Filter
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.63
axis_max = 1
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()

passthrough = passthrough.make_passthrough_filter()
filter_axis = 'x'
passthrough.set_filter_field_name(filter_axis)
x_axis_min = 0.35
x_axis_max = 1
passthrough.set_filter_limits(x_axis_min, x_axis_max)
cloud_filtered = passthrough.filter()
pcl.save(cloud_filtered, 'passf.pcd')
```

![Passthrough filter][passf]

**RANSAC Plane Segmentation**
```python
# RANSAC Plane Segmentation
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.01

# Extract inliers and outliers
inliers, coefficients = seg.segment()
cloud_table = cloud_filtered.extract(inliers, negative=False)
cloud_objects = cloud_filtered.extract(inliers, negative=True)
```

![RANSAC Plane Segmentation][ransacf]

* Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

**Euclidean Clustering**
```python
# Euclidean Clustering
white_cloud = XYZRGB_to_XYZ(cloud_objects)
tree = white_cloud.make_kdtree()

# Create Cluster-Mask Point Cloud to visualize each cluster separately
# Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
# Set tolerances for distance threshold
# as well as minimum and maximum cluster size (in points)
# NOTE: These are poor choices of clustering parameters
# Your task is to experiment and find values that work for segmenting objects.
ec.set_ClusterTolerance(0.01)
ec.set_MinClusterSize(50)
ec.set_MaxClusterSize(750)
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()
```

![Euclidean clustering][clusters]

* Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

**Dataset generation**
I generated a single dataset containing the objects required in all three scenarios. To do this I modified `model` variable inside `capture_features.py`. This was part of `sensor_stick` project.

```python
models = [\
   'sticky_notes',
   'book',
   'snacks',
   'biscuits',
   'eraser',
   'soap2',
   'soap',
   'glue']
```

For each object I've spawned 150 positions to capture HSV and normal distribution histograms. Each of them with 128 bins.

**SVM model training**
To train the model using the previously generated dataset, I used `train_svm.py` script also from `sensor_stick`.

As seen in the images below, the results are very accurate. The worst result was got when recognizing the glue, which fails in 7% of the attempts.

![SVM model][svm_model]

**Object recognition**
To identify the objects in the scene I used the previously generated model and compared the features of each detected cluster with it:

```python
for index, pts_list in enumerate(cluster_indices):
    # Grab the points for the cluster
    pcl_cluster = cloud_objects.extract(pts_list)

    # Compute the associated feature vector
    ros_cluster = pcl_to_ros(pcl_cluster)
    chists = compute_color_histograms(ros_cluster, using_hsv=True)
    normals = get_normals(ros_cluster)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, nhists))

    # Make the prediction based on model.sav
    prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
    label = encoder.inverse_transform(prediction)[0]
    detected_objects_labels.append(label)

    # Add the detected object to the list of detected objects.
    do = DetectedObject()
    do.label = label
    do.cloud = ros_cluster
    detected_objects.append(do)
```

![Object recognition][world1]

## Pick and Place Setup

* For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

**World 1**

| Objects in scene | Recognized |
|------------------|------------|
| Biscuits         | Yes        |
| Soap             | Yes        |
| Soap2            | Yes        |

- Detailed results available [here](results/world1/world1.yaml).
- Accuracy: 5/5 (100%)

![Results world 1][world1]

**World 2**

| Objects in scene | Recognized |
|------------------|------------|
| Biscuits         | Yes        |
| Soap             | Yes        |
| Soap2            | Yes        |
| Glue             | Yes        |
| Book             | No         |

- Detailed results available [here](results/world2/world2.yaml).
- Accuracy: 4/5 (80%)

![Results world 2][world2]


**World 3**

| Objects in scene | Recognized |
|------------------|------------|
| Biscuits         | Yes        |
| Soap             | Yes        |
| Glue             | Yes        |
| Book             | Yes        |
| Sticky_notes     | Yes        |
| Snacks           | Yes        |
| Eraser           | No         |
| Soap2            | No         |

- Detailed results available [here](results/world2/world3.yaml).
- Accuracy: 6/8 (75%)

![Results world 3][world3]

## Conclusions
The results are good enough to pass the submission, however I was expecting better results. One of the drawbacks of ROS is that to gain flexibility it is very CPU hungry (running it inside a VM doesn't help), so neither increasing the number of points in the cloud nor increasing the number of training samples was a viable option in my computer. Nevertheless, I detected something when analyzing the results which lead me to improve the results:

![RANSAC failure][improvements]

As can be seen in the image on the left side, all the objects are recognizable by the human eye after the passthrough filter. However, after the next step (the RANSAC segmentation), used to filter out the table, important parts of the objects to be detected are removed too (see for instance the biscuits box). This results can also be seen in the rest of the scenarios. Take for instance the world one shown in the first part of this writeup; the cluster corresponding to the biscuits barely contains points!

Since the table was filtered out for the most part with the passthrough filter, reducing the maximum distance parameter of the RANSAC filter to 0.001 m helped to improve the results:

![RANSAC improvement][improvements2]

As it can be seen above, the biscuits case is complete now, though the book, which is one of the objects which the code fails to recognize is missing a significant piece. This means that playing with the different parameters a bit more I could improved the results.

In the scenario two the book is recognized as biscuits, but it's not clear to me why. Even if the normals distribution may be similar, the colors distribution should be quite different. In the third scenario the biscuits are also problematic. Increasing the number of training samples for this object may help to fix this.

To sum up, this project has been really interesting because it mixes classic and modern image processing techniques. However, a more flexible environment is needed to perform changes faster and try new ideas or modify parameters without wasting too much time.
