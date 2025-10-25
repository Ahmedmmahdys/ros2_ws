#ifndef MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_INTERFACE_IMPROVED_H
#define MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_INTERFACE_IMPROVED_H

#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit
{
namespace planning_interface
{
/// Alias retained for compatibility with repositories that expect the
/// historical MoveGroupInterfaceImproved header from the ROS 1 crane stack.
using MoveGroupInterfaceImproved = MoveGroupInterface;
}  // namespace planning_interface
}  // namespace moveit

#endif  // MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_INTERFACE_IMPROVED_H
