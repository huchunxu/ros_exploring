#include <ros/ros.h>
// 包含moveit的API
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "check_collision");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 加载机器人的运动学模型到情景实例中
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // 自身碰撞检测
  // 首先需要创建一个碰撞检测的请求对象和响应对象，然后调用碰撞检测的API checkSelfCollision，检测结果会放到collision_result中
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("1. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");

  // 修改机器人的状态
  // 我们可以使用场景实例的getCurrentStateNonConst()获取当前机器人的状态，然后修改机器人的状态到一个随机的位置，
  // 清零collision_result的结果后再次检测机器人是否发生滋生碰撞
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("2. Self collision Test(Change the state): "<< (collision_result.collision ? "in" : "not in"));

  // 我们也可以指定查询一个group是否和其他部分发生碰撞，只需要在collision_request中修改group_name属性
  collision_request.group_name = "arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("3. Self collision Test(In a group): "<< (collision_result.collision ? "in" : "not in"));

  // 获取碰撞关系
  // 首先，我们先让机器人发生自身碰撞 
  std::vector<double> joint_values;
  const robot_model::JointModelGroup* joint_model_group =
  current_state.getJointModelGroup("arm");
  current_state.copyJointGroupPositions(joint_model_group, joint_values);
  joint_values[2] = 1.57; //原来的代码这里是joint_values[0]，并不会导致碰撞，我改成了joint_values[2]，在该状态下机器人会发生碰撞
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("4. Collision points "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  // 然后我们再来检测机器人是否发生了自身碰撞，已经发生碰撞的是哪两个部分
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("5. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");

  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("6 . Contact between: %s and %s",
             it->first.first.c_str(),
             it->first.second.c_str());
  }

  // 修改允许碰撞矩阵（Allowed Collision Matrix，acm)
  // 我们可以通过修改acm来指定机器人是否检测自身碰撞和与障碍物的碰撞，在不检测的状态下，即使发生碰撞，检测结果也会显示未发生碰撞
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();
  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for(it2 = collision_result.contacts.begin();
      it2 != collision_result.contacts.end();
      ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);

  ROS_INFO_STREAM("6. Self collision Test after modified ACM: "<< (collision_result.collision ? "in" : "not in")
                  << " self collision");

  // 完整的碰撞检测
  // 同时检测自身碰撞和与障碍物的碰撞
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);

  ROS_INFO_STREAM("6. Full collision Test: "<< (collision_result.collision ? "in" : "not in")
                  << " collision");

  ros::shutdown();
  return 0;
}
