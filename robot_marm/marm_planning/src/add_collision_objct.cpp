// 包含API的头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_collision_objct");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "arm_cylinder";

    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;

    // 设置障碍物的位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.0;
    pose.position.y = -0.4;
    pose.position.z =  0.4;

    // 将障碍物的属性、位置加入到障碍物的实例中
    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;

    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    current_scene.addCollisionObjects(collision_objects);

    ros::shutdown();

    return 0;
}
