
#ifndef POSE_SELECTOR_H
#define POSE_SELECTOR_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
# include <rviz/display_group.h>

// Compiled .ui file
# include <ui_pose_selector.h>
#include "pose_reader.h"
#endif


class QComboBox;

namespace pose_selector
{

class PoseSelector: public rviz::Panel
{

// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:

  PoseSelector( QWidget* parent = 0 );

  void updateTree();


public Q_SLOTS:

  void packageChanged( int index );
  void treeClicked(QTreeWidgetItem* item, int column);
  void publishMarkers();
  void refreshClicked();


protected:

  Ui::PoseSelectorWidget ui_;

  PoseReader* pose_reader;
  Task* current_task = NULL;
  std::string package_path;

  ros::Publisher marker_pub_, pose_pub_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

}

#endif 
