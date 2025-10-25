#ifndef GAZEBO_LINK_ATTACHER_HPP_
#define GAZEBO_LINK_ATTACHER_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

struct JointSTRUCT
{
  std::string model1;
  gazebo::physics::ModelPtr m1;
  std::string link1;
  gazebo::physics::LinkPtr l1;
  std::string model2;
  gazebo::physics::ModelPtr m2;
  std::string link2;
  gazebo::physics::LinkPtr l2;
  gazebo::physics::JointPtr joint;
};

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate;

class GazeboLinkAttacher : public gazebo::WorldPlugin
{
public:
  
  // Constructor:
  GazeboLinkAttacher();

  // Destructor:
  virtual ~GazeboLinkAttacher();

  // LOAD plugin:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:

  std::unique_ptr<GazeboLinkAttacherPrivate> impl_;

};

}  // namespace gazebo_ros

#endif  // GAZEBO_LINK_ATTACHER_HPP_