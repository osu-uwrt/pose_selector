
#ifndef POSE_READER_H
#define POSE_READER_H

#ifndef Q_MOC_RUN
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#endif

namespace pose_selector
{


class Pose
{
public:
  Pose(std::string name, std::vector<float> values);
  geometry_msgs::Pose getPoseMsg();

  std::string name;

protected:
  float x, y, z, roll, pitch, yaw;
};


class Task
{

public:
  Task(std::string name);
  void addPose(Pose* pose);


  std::string name;
  std::vector<Pose*> pose_list;

};



class PoseReader
{

public:
  PoseReader( std::string package_path );

  std::vector<Task*> task_list;

protected:

  std::string package_path;
  
};

} 

#endif
