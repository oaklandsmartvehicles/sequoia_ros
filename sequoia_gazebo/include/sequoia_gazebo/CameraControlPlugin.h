#ifndef CAMERACONTROLPLUGIN_H_
#define CAMERACONTROLPLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sequoia_gazebo/CameraControlPluginConfig.h>
#include <tf/tf.h>

namespace gazebo
{

class CameraControlPlugin : public SystemPlugin
{
public:
  CameraControlPlugin();
  virtual ~CameraControlPlugin();
protected:
  virtual void Load(int argc, char** argv);
  virtual void Init();
  virtual void Reset();
private:
  void reconfig(sequoia_gazebo::CameraControlPluginConfig& config, uint32_t level);
  void Update();
  void lookAtRobot(const rendering::VisualPtr& camera_target);

  rendering::UserCameraPtr user_cam_;
  std::vector<event::ConnectionPtr> connections_;
  ros::AsyncSpinner* async_;
  ros::NodeHandle* n_;
  boost::shared_ptr<dynamic_reconfigure::Server<sequoia_gazebo::CameraControlPluginConfig> > srv_;
  sequoia_gazebo::CameraControlPluginConfig cfg_;
  math::Vector3 camera_pos_;
};

GZ_REGISTER_SYSTEM_PLUGIN(CameraControlPlugin)
}

#endif // CAMERACONTROLPLUGIN_H_
