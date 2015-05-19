#ifndef _SPHERE_H_
#define _SPHERE_H_

#include <openrave/openrave.h>

namespace orchomp{

class CollisionDataCache;

class Sphere
{
  public:
    // The radius of the sphere
    double radius;
    
    bool inactive;

    //a pointer to the kinBody that
    //the sphere comes off of.
    OpenRAVE::KinBody::Link * link;
    
    OpenRAVE::KinBody * body;

    //the name of the kinbody it is linked to;
    std::string linkname; 
    
    CollisionDataCache * cache;
    
    //the index of the robot link
    int linkindex;

    //The transform between the coordinates of the
    //  robot link and the sphere. Since it is a sphere,
    //  rotation is meaningless, so it should just be an
    //  xyz vector.
    OpenRAVE::Vector position;

    Sphere() : inactive(false), link( NULL ), cache( NULL), linkindex(-1){}
};

}//namespace orchomp

#endif 
