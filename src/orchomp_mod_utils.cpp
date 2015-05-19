
#include "orchomp_mod.h"
#include "orchomp_kdata.h"

namespace orchomp
{

bool mod::isWithinPaddedLimits( const chomp::MatX & mat ) const{
    assert( upperJointLimits.size() > 0 );
    assert( lowerJointLimits.size() > 0 );
    assert( mat.cols() > 0 );

    for ( int i = 0; i < mat.cols(); i ++ ){
        if ( mat(i) > paddedUpperJointLimits[i] ||
             mat(i) < paddedLowerJointLimits[i] ){
            return false;
        }
    }
    return true;
}

bool mod::isWithinLimits( const chomp::MatX & mat ) const{
    assert( upperJointLimits.size() > 0 );
    assert( lowerJointLimits.size() > 0 );
    assert( mat.cols() > 0 );

    for ( int i = 0; i < mat.cols(); i ++ ){
        if ( mat(i) > upperJointLimits[i] ||
             mat(i) < lowerJointLimits[i] ){
            return false;
        }
    }
    return true;
}

void mod::coutTrajectory() const
{
    for ( int i = 0; i < chomper->xi.rows(); i ++ ){


        for ( int j = 0; j < chomper->xi.cols() ; j ++ ){

            std::cout << chomper->xi( i, j ) << "\t";
        }
        std::cout << "\n";
    }
}

void mod::isTrajectoryWithinLimits() const {
    for( int i = 0; i < chomper->xi.rows(); i ++ ){
        chomp::MatX test = chomper->xi.row(i);
        assert( isWithinLimits( test ) );
        //debugStream << "Point " << i << " is within limits" << std::endl;
    }
}


void mod::getIK( const OpenRAVE::Transform & xform, 
                 std::vector< OpenRAVE::dReal > & solution ){

    OpenRAVE::IkParameterization param;
    param.SetTransform6D( xform );

    active_manip->FindIKSolution( param, solution, 
                                  OpenRAVE::IKFO_CheckEnvCollisions);

}

void mod::clampToLimits( chomp::MatX & state ){
    for ( int i = 0; i < state.cols() ; i ++ ){
        if ( state(i) > paddedUpperJointLimits[i] ){
            state(i) = paddedUpperJointLimits[i];
        }
        else if ( state(i) < paddedLowerJointLimits[i] ){
            state(i) = paddedLowerJointLimits[i];
        }
    }
}


void mod::vectorToMat(const std::vector< OpenRAVE::dReal > & vec,
                             chomp::MatX & mat )
{
    assert( vec.size() > 0 );
    mat.resize(1, vec.size() );

    for ( size_t i = 0; i < vec.size() ; i ++ ){ mat(i) = vec[i]; }
}


void mod::getStateAsVector( const chomp::MatX & state,
                                   std::vector< OpenRAVE::dReal > & vec ){

    vec.resize( n_dof );
    assert( state.size() == int( n_dof ) );
    
    for ( size_t i = 0; i < n_dof; i ++ ){
        vec[i] = state(i);
    }
}


void mod::getIthStateAsVector( size_t i, 
                      std::vector< OpenRAVE::dReal > & state )
{
    
    const int width = chomper->xi.cols();
    state.resize( width );

    for ( int j = 0; j < width; j ++ ){
        state[j] = chomper->xi(i, j );
    }

}

void mod::setActiveDOFValues( const chomp::MatX & qt ){

    std::vector< OpenRAVE::dReal > vec;
    getStateAsVector( qt, vec );

    robot->SetActiveDOFValues( vec, false );
}

bool mod::areAdjacent( int first, int second ) const {

    //second must be larger than first, so if that is not the case, swap
    //  them.
    if ( first > second ){ std::swap( first, second ); }
    
    //for some reason, openRAVE uses this idiotic structure
    //  to store adjacency values.
    int value = ( second << 16 ) | (first & 0x0000FFFF);
    std::set<int>::iterator it = robot->GetAdjacentLinks().find(value);
    
    //if the iterator does not hold an item, 
    //  then the links are not adjacent
    if ( robot->GetAdjacentLinks().end() == it ){
        return false;
    }
    return true;
}



OpenRAVE::KinBodyPtr mod::createBox( const OpenRAVE::Vector & pos,
                                const OpenRAVE::Vector & extents,
                                const OpenRAVE::Vector & color,
                                float transparency)
{


    OpenRAVE::KinBodyPtr cube = RaveCreateKinBody( environment );
    
    std::stringstream ss;
    ss   << pos[0] << "_"
         << pos[1] << "_"
         << pos[2] ; 

    const std::string name = ss.str();

    if( environment->GetKinBody( name.c_str() ).get() ){
        debugStream << "That cube already exists" << std::endl;
        return cube;
    }
    cube->SetName( name.c_str() );

    //set the dimensions of the cube object
    std::vector< OpenRAVE::AABB > vaabbs(1);

    /* extents = half side lengths */
    vaabbs[0].extents = extents;
    vaabbs[0].pos = pos;
    cube->InitFromBoxes(vaabbs, true);
    
    //add the cube to the environment
    environment->Add( cube );

    cube->GetLinks()[0]->GetGeometries()[0]->SetAmbientColor( color );
    cube->GetLinks()[0]->GetGeometries()[0]->SetDiffuseColor( color );
    cube->GetLinks()[0]->GetGeometries()[0]->SetTransparency(transparency);
    
    return cube;

}

} //namespace
