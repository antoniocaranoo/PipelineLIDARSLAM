# IRA SLAM Node (ROS2, C++)

Questo nodo ROS2 implementa la funzionalità di place recognition basato su ScanContext

# Repository di riferimento ScanContext

https://github.com/gisbi-kim/scancontext_tro

## Dipendenze

- ROS2 
- `pcl` e `pcl_ros`
- `scancontext` (libreria esterna per la descrizione delle scene)
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `rclcpp`

## Funzionamento

Il nodo:

1. Sottoscrive un topic di nuvole di punti (/pointcloud_topic)
2. Sottoscrive un topic di odometria (/kiss/odometry)
3. Quando il robot si è spostato di almeno 1 metro, genera un 'descriptor scan context' dalla point cloud.
4. Confronta la nuova descrizione con le precedenti per rilevare chiusure di loop.

## File Principali

- `subscriber_node.cpp`: implementazione del nodo principale in ROS2
- Richiede la classe `SCManager` dalla libreria `scancontext`

## Esecuzione

```bash
colcon build --packages-select ira_slam_node
source install/setup.bash
ros2 run ira_slam_node ira_slam_node --ros-args --params-file install/ira_slam_node/share/ira_slam_node/config/params.yaml