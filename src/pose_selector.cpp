
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTreeWidgetItem>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>


#include "pose_selector.h"

namespace pose_selector
{

PoseSelector::PoseSelector( QWidget* parent )
  : rviz::Panel( parent )
{
  ui_.setupUi(this);

  // Create our publishers
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 5 );
  pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>( "pose_array", 5 );

  // Timer to publish our messages
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( publishMarkers() ));
  connect( ui_.package_selector_widget, SIGNAL( currentIndexChanged( int )), this, SLOT(packageChanged(int)));
  connect( ui_.pose_tree_widget, SIGNAL( itemClicked(QTreeWidgetItem*, int) ), this, SLOT(treeClicked(QTreeWidgetItem*, int)));
  connect( ui_.refresh_button, SIGNAL( clicked() ), this, SLOT(refreshClicked()));
  
  // Find all our competition packages
  ros::package::V_string packages;
  ros::package::getAll(packages);
  for(std::string package : packages) {
    if (package.find("riptide_robosub") != std::string::npos) {
      ui_.package_selector_widget->addItem(QString::fromStdString(package));
    }
  }

  // Start the publishing timer. 10Hz
  output_timer->start( 100 );
}

// When refresh button is clicked, update the tree and read file.
void PoseSelector::refreshClicked( )
{
  packageChanged(0);
}

// When package is changes, read files
void PoseSelector::packageChanged( int index )
{
  std::string package = ui_.package_selector_widget->currentText().toUtf8().constData();
  if (package != "")
  {
    package_path = ros::package::getPath(package);
    pose_reader = new PoseReader(package_path);
    updateTree();
  }
}

// When tree is clicked, update the current task
void PoseSelector::treeClicked(QTreeWidgetItem* item, int column)
{
  int task_number;
  QModelIndex index = ui_.pose_tree_widget->currentIndex();
  if (index.parent().isValid())
    task_number = index.parent().row();
  else
    task_number = index.row();
    
  current_task = pose_reader->task_list.at(task_number);
}


// Fill the tree with data from package
void PoseSelector::updateTree()
{
  ui_.pose_tree_widget->clear();
  for (Task* task: pose_reader->task_list)
  {
    QTreeWidgetItem *treeItem = new QTreeWidgetItem(ui_.pose_tree_widget);

    // QTreeWidgetItem::setText(int column, const QString & text)
    treeItem->setText(0, QString::fromStdString(task->name));
    treeItem->setText(1, "");
    for (Pose* pose: task->pose_list)
    {
      QTreeWidgetItem *childItem = new QTreeWidgetItem(treeItem);

      // QTreeWidgetItem::setText(int column, const QString & text)
      childItem->setText(0, QString::fromStdString(pose->name));
      childItem->setText(1, "");
    }
  }
}

// Publish marker and pose array
void PoseSelector::publishMarkers()
{
  if (current_task != NULL)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0; // Alpha 0 so texture showh through
    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "file://"+package_path+"/models/"+current_task->name+"/meshes/"+current_task->name+".dae";
    marker_pub_.publish( marker );


    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "world";
    poses.header.stamp = ros::Time();
  
    for (Pose* pose: current_task->pose_list)
    {
      poses.poses.push_back(pose->getPoseMsg());
    }
    pose_pub_.publish(poses);
  }
}

} 

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pose_selector::PoseSelector,rviz::Panel )

