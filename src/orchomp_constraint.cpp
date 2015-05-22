#include "orchomp_constraint.h"
#include "orchomp_mod.h"

namespace orchomp

{

ORTSRConstraint::ORTSRConstraint(mod * module, int manip_index,
                                 mopt::Transform & pose_0_w, 
                                 mopt::MatX & Bw,
                                 mopt::Transform & pose_w_e,
                                 std::string body_name,
                                 std::string link_name) :
     TSRConstraint( pose_0_w, Bw, pose_w_e ),
     module( module ), 
     body_name( body_name ), link_name( link_name )
{
    
    if ( manip_index >= 0 ){
        const std::vector< OpenRAVE::RobotBase::ManipulatorPtr > & manips = 
                                    module->robot->GetManipulators();

        //if the manip is outside of the correct range, throw an error
        if ( int( manips.size() ) <= manip_index ){
            RAVELOG_ERROR( "There is no manipulator with index: %d\n",
                            manip_index );
            throw OpenRAVE::openrave_exception( "Bad arguments!");
        }else {
            ee_link_index = manips[manip_index]->GetEndEffector()
                                               ->GetIndex();
        }

    }
    
    else if (body_name != "NULL" && link_name != "NULL" ){
        OpenRAVE::KinBodyPtr body = module->environment
                                          ->GetKinBody(body_name);
        if (body.get()){
            OpenRAVE::KinBody::LinkPtr link = body->GetLink(link_name);
            if (link.get() ){
                ee_link_index = link->GetIndex();
            }
            else {
                RAVELOG_ERROR("There is no link named |%s| on body |%s|\n",
                              link_name.c_str(), body_name.c_str() );
                throw OpenRAVE::openrave_exception( "Bad arguments!");
            }
        }

        else {
            RAVELOG_ERROR( "There is no body named: %s\n",
                            link_name.c_str() );
            throw OpenRAVE::openrave_exception( "Bad arguments!");
        }
    }

    //handle the case when both are invalid
    else {
        RAVELOG_ERROR( "There is no valid manipulator index or"
                       " valid body and link name\n" );
        throw OpenRAVE::openrave_exception( "Bad arguments!");
    }

}


//this function takes in a robot state, qt, and returns the position of
// the relevant end-effector in the world frame. This is equivalent
//  to the transformation from the end-effector frame to the world
//  frame.
void ORTSRConstraint::forwardKinematics( const mopt::MatX& qt,
                                         mopt::Transform & pos )
{

    module->setActiveDOFValues( qt);
    OpenRAVE::Transform t = module->robot->GetLinks()[ee_link_index]
                                  ->GetTransform();

    pos.setTranslation( vec3( t.trans.x, t.trans.y, t.trans.z ) );
    pos.setRotation( mopt::Transform::quat(
                     t.rot.x, t.rot.y, t.rot.z, t.rot.w ) );
    

}

//this takes in a state and it gets the jacobian 
void ORTSRConstraint::computeJacobian( 
                              const mopt::MatX& qt,
                              const mopt::Transform & pose_world_ee,
                              mopt::MatX & jacobian,
                              std::vector< int > & active_dims)
{


    //make sure that there are constrained dimensions, if not, return
    if ( active_dims.size() <= 0 ){ return; }
    
    //get the degrees of freedom
    const int DOF = qt.size();
    
    OpenRAVE::Transform t = module->robot->GetLinks()[ ee_link_index ]
                                  ->GetTransform();
    std::vector< OpenRAVE::dReal > translationJacobian;
    std::vector< OpenRAVE::dReal > rotationJacobian;

    //get the jacobians 
    for ( size_t i = 0; i < active_dims.size(); i++ ){
        if ( active_dims[i] < 3 && translationJacobian.size() == 0){
            module->robot->CalculateActiveJacobian( ee_link_index,
                                                t.trans,
                                                translationJacobian);

            assert( translationJacobian.size() == size_t( DOF * 3 ));
        }
        else if ( active_dims[i] >= 3 && rotationJacobian.size() == 0 ){
            module->robot->CalculateActiveRotationJacobian( ee_link_index,
                                                t.rot,
                                                rotationJacobian);
            assert( rotationJacobian.size() == size_t( DOF * 3 ) );
        }
    }
    
    //copy the jacobian data into the jacobian matrix
    for ( size_t i = 0; i < active_dims.size(); i ++ ){

        //extract the current index
        const int j = active_dims[i];
        
        for ( int k = 0; k < DOF; k ++ ){

            //if j is less than 3, then the dimension is a translation
            //  dimension, so the translation jacobian is copied
            if ( j < 3 ){
                assert( size_t( j * DOF + k ) < translationJacobian.size());
                jacobian(i, k) = translationJacobian[ j * DOF + k ];
            }
            else {
                assert( size_t((j-3) * DOF + k) < rotationJacobian.size());
                jacobian(i, k) = rotationJacobian[ (j-3) * DOF + k ];
            }
        }
    }
}




}// namespace
