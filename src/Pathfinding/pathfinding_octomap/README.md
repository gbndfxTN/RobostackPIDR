# Pathfinding OctoMap (GO2 RGBD)

Ce package lance `octomap_server` a partir du nuage de points RGBD du robot GO2.

## Entrees / sorties

- Entree attendue (par defaut): `/camera/depth/color/points` (`sensor_msgs/msg/PointCloud2`)
- Sorties principales:
  - `/octomap_binary`
  - `/octomap_full`
  - `/octomap_point_cloud_centers`

## Build

Depuis la racine du workspace:

```bash
pixi run colcon build --symlink-install --packages-select pathfinding_octomap
```

## Lancement OctoMap

```bash
pixi run ros2 launch pathfinding_octomap octomap_from_rgbd.launch.py
```

## Exemples utiles

Topic de nuage different:

```bash
pixi run ros2 launch pathfinding_octomap octomap_from_rgbd.launch.py cloud_in_topic:=/camera/depth_registered/points
```

Resolution a 2 cm:

```bash
pixi run ros2 launch pathfinding_octomap octomap_from_rgbd.launch.py resolution:=0.02
```

Portee max capteur a 4 m:

```bash
pixi run ros2 launch pathfinding_octomap octomap_from_rgbd.launch.py max_range:=4.0
```

## Verification rapide

```bash
pixi run ros2 topic list | grep octomap
pixi run ros2 topic echo /octomap_binary --once
```

Dans RViz2, ajoute l'affichage `Map` (topic `/octomap_binary`) ou `PointCloud2` (topic `/octomap_point_cloud_centers`).
