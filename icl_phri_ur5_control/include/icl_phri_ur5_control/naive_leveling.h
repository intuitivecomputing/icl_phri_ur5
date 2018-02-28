#ifndef NAIVE_LEVELING_H
#define NAIVE_LEVELING_H

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <boost/shared_ptr.hpp>

namespace ur5_control
{
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;

class NaiveLeveling
{
public:
    NaiveLeveling();
    virtual ~NaiveLeveling();
private:

}
} // namespace ur5_control


#endif