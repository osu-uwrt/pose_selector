#include "pose_reader.h"
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/filesystem.hpp>
#include <math.h>

namespace pose_selector
{

Pose::Pose ( std::string name, std::vector<float> values ){
    this->name = name;
    x = values.at(0);
    y = values.at(1);
    z = values.at(2);
    roll = values.at(3) * M_PI / 180;
    pitch = values.at(4) * M_PI / 180;
    yaw = values.at(5) * M_PI / 180;
}

// Create a pose message representing this pose
geometry_msgs::Pose Pose::getPoseMsg (){
    geometry_msgs::Pose msg;
    this->name = name;

    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );

    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.orientation.x = quaternion.getX();
    msg.orientation.y = quaternion.getY();
    msg.orientation.z = quaternion.getZ();
    msg.orientation.w = quaternion.getW();

    return msg;
}

Task::Task( std::string name ){
    this-> name = name;
}

void Task::addPose( Pose* pose )
{
    this->pose_list.push_back(pose);
}

// Read data from pose file
PoseReader::PoseReader( std::string package_path )
{
    // Load in all tasks
    this->package_path = package_path;
    std::string task_path = package_path+"/cfg/tasks.yaml";

    if (boost::filesystem::exists(task_path))
    {
        YAML::Node config = YAML::LoadFile(task_path);
        for (YAML::Node item: config) {
            Task* task = new Task(item.as<std::string>());
            task_list.push_back(task);
        }
    } else {
        ROS_ERROR("Task file %s does not exist.", task_path.c_str());
    }

    // Read task poses
    std::string pose_path = package_path+"/cfg/task_poses.yaml";
    if (boost::filesystem::exists(pose_path))
    {
        YAML::Node config = YAML::LoadFile(pose_path);
        for (Task* task: task_list)
        {
            if(config[task->name])
            {
                YAML::Node task_yaml = config[task->name];
                for(YAML::const_iterator it=task_yaml.begin();it != task_yaml.end();++it) {
                    Pose* pose = new Pose(
                        it->first.as<std::string>(), 
                        it->second.as<std::vector<float>>()
                    );
                    task->addPose(pose);
                }
            }
        }
    }
}

}
