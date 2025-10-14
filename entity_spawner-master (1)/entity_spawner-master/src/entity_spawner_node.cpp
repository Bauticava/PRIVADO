#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <boost/variant.hpp>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_operations.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "entity_spawner/srv/load_entity.hpp"
#include "entity_spawner/srv/spawn_entity.hpp"

namespace entity_spawner
{

struct Entity
{
  std::string name;
  std::string source_path;
  shape_msgs::msg::Mesh mesh;
};

shape_msgs::msg::Mesh scaleMesh(const shape_msgs::msg::Mesh& mesh, double scale)
{
  shape_msgs::msg::Mesh scaled_mesh = mesh;
  for (auto& vertex : scaled_mesh.vertices)
  {
    vertex.x *= scale;
    vertex.y *= scale;
    vertex.z *= scale;
  }
  return scaled_mesh;
}

class EntitySpawnerNode : public rclcpp::Node
{
public:
  EntitySpawnerNode()
  : rclcpp::Node("entity_spawner_node")
  {
    load_entity_service_ = this->create_service<entity_spawner::srv::LoadEntity>(
      "loadEntity",
      std::bind(&EntitySpawnerNode::handleLoadEntity, this, std::placeholders::_1, std::placeholders::_2));

    spawn_entity_service_ = this->create_service<entity_spawner::srv::SpawnEntity>(
      "spawnEntity",
      std::bind(&EntitySpawnerNode::handleSpawnEntity, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "EntitySpawnerNode ready to load and spawn entities.");
  }

private:
  void handleLoadEntity(
    const std::shared_ptr<entity_spawner::srv::LoadEntity::Request> request,
    std::shared_ptr<entity_spawner::srv::LoadEntity::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entity_name = request->entity_name;
    if (entity_name.empty())
    {
      response->success = false;
      response->message = "Entity name must not be empty.";
      return;
    }

    if (entity_cache_.count(entity_name) != 0)
    {
      response->success = false;
      response->message = "Entity name already loaded.";
      return;
    }

    std::error_code ec;
    auto resolved_path = std::filesystem::absolute(request->stl_path, ec);
    if (ec)
    {
      response->success = false;
      response->message = "Unable to resolve STL path: " + ec.message();
      return;
    }

    const std::string resolved_path_str = resolved_path.string();

    if (!std::filesystem::exists(resolved_path))
    {
      response->success = false;
      response->message = "STL file does not exist: " + resolved_path_str;
      return;
    }

    if (!resolved_path.has_extension() || resolved_path.extension() != ".stl")
    {
      response->success = false;
      response->message = "Provided file is not an .stl model: " + resolved_path_str;
      return;
    }

    const std::string resource_uri = "file://" + resolved_path_str;

    shapes::Mesh* mesh_ptr = shapes::createMeshFromResource(resource_uri);
    if (!mesh_ptr)
    {
      response->success = false;
      response->message = "Failed to parse STL mesh: " + resolved_path_str;
      return;
    }

    shapes::ShapePtr shape_ptr(mesh_ptr);
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(shape_ptr.get(), shape_msg);

    shape_msgs::msg::Mesh mesh_msg;
    try
    {
      mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
    }
    catch (const boost::bad_get& /*ex*/)
    {
      response->success = false;
      response->message = "Loaded shape is not a mesh: " + resolved_path_str;
      return;
    }

    Entity entity;
    entity.name = entity_name;
    entity.source_path = resolved_path_str;
    entity.mesh = mesh_msg;

    entity_cache_.emplace(entity_name, std::move(entity));

    response->success = true;
    response->message = "Entity loaded and queued: " + entity_name;

    RCLCPP_INFO(
      get_logger(), "Loaded entity '%s' from '%s'", entity_name.c_str(), resolved_path_str.c_str());
  }

  void handleSpawnEntity(
    const std::shared_ptr<entity_spawner::srv::SpawnEntity::Request> request,
    std::shared_ptr<entity_spawner::srv::SpawnEntity::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = entity_cache_.find(request->entity_name);
    if (it == entity_cache_.end())
    {
      response->success = false;
      response->message = "Entity not found in queue: " + request->entity_name;
      return;
    }

    if (request->frame_id.empty())
    {
      response->success = false;
      response->message = "Frame id must not be empty.";
      return;
    }

    if (request->scale <= 0.0)
    {
      response->success = false;
      response->message = "Scale must be greater than zero.";
      return;
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = request->entity_name;
    collision_object.header.frame_id = request->frame_id;

    collision_object.meshes.push_back(scaleMesh(it->second.mesh, request->scale));

    geometry_msgs::msg::Pose pose;
    pose.position.x = request->x;
    pose.position.y = request->y;
    pose.position.z = request->z;

    tf2::Quaternion orientation;
    orientation.setRPY(request->roll, request->pitch, request->yaw);
    pose.orientation = tf2::toMsg(orientation);

    collision_object.mesh_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    const bool applied = planning_scene_interface_.applyCollisionObject(collision_object);
    if (!applied)
    {
      response->success = false;
      response->message = "Failed to apply collision object to planning scene.";
      return;
    }

    response->success = true;
    response->message = "Entity spawned: " + request->entity_name;

    RCLCPP_INFO(
      get_logger(),
      "Spawned entity '%s' at (%.3f, %.3f, %.3f) frame '%s' scale %.3f",
      request->entity_name.c_str(),
      request->x,
      request->y,
      request->z,
      request->frame_id.c_str(),
      request->scale);

    entity_cache_.erase(it);
  }

  rclcpp::Service<entity_spawner::srv::LoadEntity>::SharedPtr load_entity_service_;
  rclcpp::Service<entity_spawner::srv::SpawnEntity>::SharedPtr spawn_entity_service_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::mutex mutex_;
  std::unordered_map<std::string, Entity> entity_cache_;
};

}  // namespace entity_spawner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<entity_spawner::EntitySpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
