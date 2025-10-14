# Entity Spawner

ROS 2 (Jazzy) package providing service-based management for loading and spawning STL meshes into a MoveIt planning scene.

## Services

- `loadEntity (entity_spawner/srv/LoadEntity)`
  - **Request:** `stl_path`, `entity_name`
  - **Response:** `success`, `message`
  - Validates the path, parses the STL mesh, and enqueues the entity for later spawning.
- `spawnEntity (entity_spawner/srv/SpawnEntity)`
  - **Request:** `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `scale`, `entity_name`, `frame_id`
  - **Response:** `success`, `message`
  - Applies the queued mesh (scaled by `scale`) as a collision object in the planning scene and removes it from the queue.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select entity_spawner
source install/setup.bash
```

## Run

```bash
ros2 run entity_spawner entity_spawner_node
```

## Example usage

1. Load a mesh:
   ```bash
   ros2 service call /loadEntity entity_spawner/srv/LoadEntity "{stl_path: '/absolute/path/model.stl', entity_name: 'sample_model'}"
   ```
2. Spawn it into the planning scene:
   ```bash
   ros2 service call /spawnEntity entity_spawner/srv/SpawnEntity "{x: 0.0, y: 0.0, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 0.0, scale: 0.01, entity_name: 'sample_model', frame_id: 'world'}"
   ```
