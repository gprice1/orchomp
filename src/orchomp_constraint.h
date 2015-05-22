
#ifndef _ORCHOMP_CONSTRAINT_H_
#define _ORCHOMP_CONSTRAINT_H_

#include "motionoptimizer/MotionOptimizer.h"

namespace orchomp
{

class mod;

class ORTSRConstraint : public mopt::TSRConstraint {
  public:
    mod * module;
    int ee_link_index;

    //TODO These do nothing right now.
    const std::string body_name;
    const std::string link_name;

    ORTSRConstraint( mod * module, int manip_index,
                     mopt::Transform & pose_0_w, 
                     mopt::MatX & Bw,
                     mopt::Transform & pose_w_e,
                     std::string body_name="NULL",
                     std::string link_name="NULL"); 

    //this function takes in a robot state, qt, and returns the position of
    // the relevant end-effector in the world frame. This is equivalent
    //  to the transformation from the end-effector frame to the world
    //  frame.
    virtual void forwardKinematics( const mopt::MatX& qt,
                                    mopt::Transform & pos );
    
    //this takes in a state and it gets the jacobian 
    virtual void computeJacobian( const mopt::MatX& qt,
                                  const mopt::Transform & pose_world_ee,
                                  mopt::MatX & jacobian,
                                  std::vector< int > & active_dims);
};

}//namespace

#endif
