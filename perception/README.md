# Perception Project

Congratulations for all of you to make it here. In our perception project, we will utilize those techniques we learnt
through this course to perform object detection task on the real world data. The quality of your solution will
be evaluated by using the metrics taught in the lecture.

## Goal
Implement the function `interface::perception::PerceptionObstacles Perception::OneIteration(const PointCloud& pointcloud)`
in `perception.cc`. The inputs is the pointcloud from our lidar. Your outputs should be all detected objects within certain range and organized by using
protobuf `interface::perception::PerceptionObstacles`, which you should already know well in Homework3. Note that in this project, you will need to output the type and velocity as well. Velocity is represented by speed and heading. That is on the xy plane, the velocity is (speed * cos(heading), speed * sin(heading)). Note that the time difference between two adjusted frames is 0.1s.
The detection range in evaluation is defined as 30m in a given ROI (region of interest), which means even you missed all objects beyond 30m or most than half points of it are outside ROI, your evaluation score will 
not be affected at all, Hooray!
For k-th laser_point in pointcloud, you can find whether it is in roi by querying is_in_roi[k] in pointcloud. If it is true, the point is in ROI, otherwise, it isn't. An obstacle will only be considered in evaluation if it is mostly in ROI.
Note that you will need to record track information in the class Perception to get the velocity of obstacles.

**Note, the following field in `interface::perception::PerceptionObstacles` need to be filled correctly 
in order to be evaluated properly:**
- `interface.geometry.Point3D polygon_point`: All polygon points need to be in world coordinate system.
Only **x** and **y** are required be be correct and **z** will only affect visualization.

- `ObjectType type`: The type of the object (please refer `perception.proto` for all supported object
types and you only need to care about **UNKNOWN_TYPE**, **CAR**, **CYCLIST** and **PEDESTRIAN** for this project.) Please
be aware that **UNKNOWN_TYPE** will be filtered out in evaluation so that they won't contribute to the score
directly. For example, when you detect some object which doesn't
overlap with any existing label, if you classify it as **UNKNOWN_TYPE**, your overall precision will
not be affected. However, if you classify it as any other type, you will see a drop in your metrics (See following examples for details.)

- `double heading`: The heading of an obstacle. We only care about the speed on x-y plane. If the obstacle is rotating or accelerating, we use the average velocity of the center of its bounding box between two adjusted frames as ground truth.

- `double speed`: The speed of an obstacle. `speed` along with `heading` could decide the velocity of an obstacle.


## Detection Metrics

First a detection may be correct or incorrect. A detection is correct if IOU (intersect over union) is above 0.5 and the classification result is correct. Here, IOU is for points. For example, a label contains 10 points and you detect an obstacle with 13 points that shares 9 common points with the original obstacle. As a result, the IOU is 9 / (10 + 13 - 9), which is above 0.5. If the classification result is correct, this will be counted as a good detection.

Let p denote precision and r denote recall.
For each class, the F1 score is defined as 2 * p * q / (p + q). We use the average F1 score among PEDESTRIAN, CAR and CYCLIST as your result.


### Tracking Metrics

The metric for evaluating velocity is defined as follows. Assuming there are `n` labelled obstacles, the score (smaller is better) is:

```
Score = \frac{1}{n} \sum_{l_i \in labeled\_obstacles}diff(l_i, o_i)
```
Where diff defined as follows
- If we could find an obstacle o_i whose IOU with l_i is more than 0.5 (we don't require its type to be correct), diff is the length of the difference between ground truth velocity and outputted velocity capped by 5.0.
- If we couldn't find an obstacle o_i whose IOU with l_i is more than 0.5, the diff is 5.0. 

## Run Perception Evaluation
First update the data and label path in `config/eval.config` (Please update all .config files when 
you want to run multiple scenarios), then run the following command:

```
bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir /${Path to your config foler}
```

If your config file and data folder are set properly, you will see the following output:

```
*********************************************************

Type: CAR
Precision: 0.6
Recall: 0.00977199
F1 score: 0.00961538

Type: PEDESTRIAN
Precision: 0
Recall: 0
F1 score: 0

Type: CYCLIST
Precision: 0
Recall: 0
F1 score: 0
Average F1 score: 0.00320513
Average velocity difference is 4.91954
*********************************************************
Evaluation result folder: /tmp/1555414441/

```
The program will calculate the overall precision and recall for each frame for each config file and 
output the averaged ones in the end. (All frames will have the same weight during averaging.)

## Run Perception Evaluation Viewer
In order to better understand the performance of your code, we also provide a evaluation viewer to help
you visualize the label and the object detected by your code. To run the viewer:

```
bazel run -c opt //perception:evaluation_viewer_main  -- --eval_result_file ${Evaluation result folder}/${scenario_name}.result
```

# Sample data

The data we provide contains 5000+ frames with (in total) 117676 car examples, 12062 pedestrian examples and only 627 cyclist examples.
It may be worthy to speed some time to explore how to deal with the cyclist data. 
To get better result, you may also need to make use of the labels outside ROI.

A sample data could be found at [this link](https://pan.baidu.com/s/1yZ3_2yzApoGzVRVO-5eI0A) with password rdhx
