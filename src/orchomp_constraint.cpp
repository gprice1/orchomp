#include "orchomp_constraint.h"
#include "orchomp_mod.h"

namespace orchomp

{


void UnifiedConstraint::evaluateConstraints( const chomp::MatX& qt, 
                                             chomp::MatX& h, 
                                             chomp::MatX& H)
{

    const int DoF = qt.size();

    std::vector< chomp::MatX > h_vec;
    std::vector< chomp::MatX > H_vec;
    h_vec.resize( constraints.size() );
    H_vec.resize( constraints.size() );
    
    //get all of the constraints
    num_outputs = 0;
    for ( size_t i = 0; i < constraints.size(); i ++ ){
        constraints[i]->evaluateConstraints( qt, h_vec[i], H_vec[i] );
        num_outputs += H_vec[i].rows();
    }
    
    if ( num_outputs == 0 ){
        h.resize( 0 , 0 );
        H.resize( 0, 0 );
        return;
    }
    
    //resize the main constraint vectors
    if ( h.cols() != 1 || h.rows() != num_outputs ){
        h.resize( num_outputs, 1 );
    }
    if ( H.cols() != DoF || H.rows() != num_outputs ){
        H.resize( num_outputs, qt.size() );
    }
     

    
    //construct the h and H matrices from block operations, to copy and
    //  paste the matrices in.

    int row_start = 0;
    for ( size_t i = 0; i < constraints.size(); i ++ ){
        const int current_height = h_vec[i].size();

        if ( current_height > 0 ){
            h.block( row_start, 0, current_height, 1 ) = h_vec[i];
            H.block( row_start, 0, current_height, DoF) = H_vec[i];
        }
        row_start += current_height;
    }
}

ORTSRConstraint::ORTSRConstraint(mod * module, int manip_index,
                                 chomp::Transform & pose_0_w, 
                                 chomp::MatX & Bw,
                                 chomp::Transform & pose_w_e,
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
void ORTSRConstraint::forwardKinematics( const chomp::MatX& qt,
                                         chomp::Transform & pos )
{

    module->setActiveDOFValues( qt);
    OpenRAVE::Transform t = module->robot->GetLinks()[ee_link_index]
                                  ->GetTransform();

    pos.setTranslation( vec3( t.trans.x, t.trans.y, t.trans.z ) );
    pos.setRotation( chomp::Transform::quat(
                     t.rot.x, t.rot.y, t.rot.z, t.rot.w ) );
    

}

//this takes in a state and it gets the jacobian 
void ORTSRConstraint::computeJacobian( 
                              const chomp::MatX& qt,
                              const chomp::Transform & pose_world_ee,
                              chomp::MatX & jacobian,
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


void ORJointLimitConstraint::evaluateConstraints(
                                             const chomp::MatX& qt, 
                                             chomp::MatX& h, 
                                             chomp::MatX& H)
{

    // H : num_constraints by DOF
    // h : num_constraints by 1

    //jacobain columns must be equal to n_dof
    // the rows must be equal to the number of constraints
    int k = module->n_dof;

    if ( h.cols() != 1 || h.rows() != k ){
        h.resize( k, 1 );
    }
    if ( H.cols() != k || H.rows() != k ){
        H.resize( k, k );
    }
    H.setZero();
    
    //dims is the dimensionality of the constraint.
    int dims = 0;
    for ( int i = 0; i < qt.size(); i ++ ){
        if ( qt(i) > module->paddedUpperJointLimits[i] ){
            h(dims) = qt(i) - module->paddedUpperJointLimits[i];
            H( dims, i ) = 1;
            dims++;
        }else if ( qt(i) < module->paddedLowerJointLimits[i] ){
            h(dims) = qt(i) - module->paddedLowerJointLimits[i];
            H(dims, i ) = 1;
            dims++;
        }
    }
    
    n_outputs = dims;
    if (dims != k){ 
        H.conservativeResize( dims, k );
        h.conservativeResize( dims, 1 );
    }
}

////////////////The Factory///////////////////////////////////////


ORConstraintFactory::ORConstraintFactory( mod * module ) : module( module ){

    //add joint limit constraints to the whole trajectory.
    //chomp::Constraint * c = new ORJointLimitConstraint( module );
    //addConstraint( c, 0, 1 );

}
ORConstraintFactory::~ORConstraintFactory(){
    
    while ( !constraints.empty() ){
        delete constraints.back();
        constraints.pop_back();
    }
}

void ORConstraintFactory::addConstraint( chomp::Constraint * c, 
                                         double start, double end )
{
    times.push_back( pair_d( start, end ) );
    constraints.push_back( c );
}

void ORConstraintFactory::removeConstraint( size_t index )
{
    times.erase( times.begin() + index );

    delete constraints[index];
    constraints.erase( constraints.begin() + index );

}



chomp::Constraint* ORConstraintFactory::getConstraint(size_t t, 
                                                      size_t total){

    UnifiedConstraint * unified = new UnifiedConstraint();

    const double time = double(t) / double( total );

    for ( size_t i = 0; i < times.size(); i ++ ){
        
        //if the timestep is within the time bounds,
        //  then add the constraint to the unified constraint.
        if ( times[i].first < time && times[i].second > time ){
            unified->addConstraint( constraints[i] );
        }
    }

    return unified;
}
    
void ORConstraintFactory::evaluate(
                const std::vector<chomp::Constraint*>& constraints, 
                const chomp::MatX& xi, chomp::MatX& h_tot,
                chomp::MatX& H_tot, int step)
{

    //debugStream << "Evaluating Constraints" <<std::endl; 

    size_t DoF = xi.cols();

    assert(size_t(xi.rows()) == constraints.size());

    //the number of rows in the complete matrices.
    size_t numCons = 0;
    
    //the number of total constraints,
    // and the number of timesteps we are actually looking at.
    const size_t size = constraints.size();
    size_t time_steps = 0;
    
    std::vector< chomp::MatX > H_vec, h_vec;

    //annoyingly, with the use of the step, this is the
    //  correct size of the vectors.
    H_vec.resize( (size - 1)/step + 1  );
    h_vec.resize( (size - 1)/step + 1  );
    

    
    //keeps track of the timestep, while i keeps track of the
    //  vector index.
    std::vector<size_t> constrained_timesteps;

    //get all of the jacobians and cost vectors
    for (size_t t=0, i=0; t< size; t+=step, i++) {
        chomp::Constraint* c = constraints[t];

        time_steps ++;
        
        //if the constraint does not exist, continue.
        if ( !c ){ continue; }

        c->evaluateConstraints( xi.row(t), h_vec[i], H_vec[i] );
        numCons += h_vec[i].size();
        
        if ( h_vec[i].size() != 0 ){
            assert( h_vec[i].rows() == h_vec[i].size() );
            assert( h_vec[i].cols() == 1);

            constrained_timesteps.push_back( i );
        }

    }
    
    //bail out if there are no constraints.
    if ( constrained_timesteps.size() == 0 ){
        h_tot.resize( 0,0 );
        H_tot.resize( 0,0 );

        //debugStream << "Done Evaluating Constraints" <<std::endl; 
        return;
    }

    assert( time_steps == (size - 1)/step + 1  );
    
    //h_tot is a row vector of length eqivalent to the number of
    //  constraints.
    //H_tot is a block diagonal matrix.
    // make h_tot and H_tot
    h_tot.resize( numCons, 1 );
    H_tot.resize( numCons, DoF*time_steps );
    H_tot.setZero(); 
   
    
    //since we don't which of the steps is 0, 
    int row_start = 0;
    for (size_t i=0; i < constrained_timesteps.size(); i ++) {
        
        const size_t index = constrained_timesteps[i];
        const int height = H_vec[index].rows();
        
        //set h block;
        h_tot.block(row_start, 0, height, 1 ) = h_vec[index];

        for (size_t j = 0; j < DoF; j ++ ){
            const size_t col_index = j * time_steps + index;
            H_tot.block(row_start, col_index , height, 1) = 
                                            H_vec[index].col( j );

        }
        row_start += height;
    }
    //debugStream << "Done Evaluating Constraints" <<std::endl; 
    
}


}// namespace
