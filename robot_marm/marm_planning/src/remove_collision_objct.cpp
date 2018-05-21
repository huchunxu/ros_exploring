// 包含API的头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    // 初始化ros节点
    ros::init(argc, argv, "remove_collision_objct");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    // 添加个需要删除的障碍物名称，然后通过planning scene interface完成删除
    std::vector<std::string> object_ids;
    object_ids.push_back("arm_cylinder");
    current_scene.removeCollisionObjects(object_ids);

    ros::shutdown();

    return 0;
}
