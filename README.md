# dip_mission
DIP Project: Implements advanced image processing algorithms and techniques for image recognition and trajectory tracking.

# 核心部分
## 两个ROS包 & 三个节点
## 包: `detector`
### 节点: `detector_node`
```
    文件路径: detector/src/detector.cpp

    功能概述: 该节点负责目标检测，通过订阅任务更新信息，根据任务类型从摄像头或视频文件中获取图像帧，进行颜色分割和模板匹配，最后发布检测到的矩形框信息。
```

## 包: `planner`
### 节点: `local_planner_node`
```
    文件路径: planner/src/local_planner.cpp

    功能概述: 该节点实现局部路径规划，可能使用传感器数据或地图信息，生成机器人的局部路径。
```

### 节点: `global_planner_node`
```
    文件路径: planner/src/global_planner.cpp

    功能概述: 该节点实现全局路径规划，基于全局地图信息，为机器人规划整体路径。
```