/** \file orchomp_mod_parse.cpp
 * \brief Implementation of the orchomp module, an implementation of CHOMP
 *        using libcd.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Carnegie Mellon University */

/* This module (orchomp) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 *
 * This handles all of the parsing for the mod class from orchomp_mod.h
 */
 

#include "orchomp_mod.h"
#include "orchomp_constraint.h"

#define DOPARSE 0
#define CREATEPARSE 1
namespace orchomp{

inline void mod::getRandomState( chomp::MatX & state ){
    assert( n_dof > 0 );
    if ( size_t( state.cols()) != n_dof ){
        state.resize( 1, n_dof );
    }

    for ( size_t i = 0; i < n_dof; i ++ ){
         const double rand_val = double( rand() ) / double(RAND_MAX);
         state(i) = lowerJointLimits[i] 
                    + (upperJointLimits[i]-lowerJointLimits[i])
                    * rand_val;
    }
}
         
void parseTransform(std::istream& sinput, OpenRAVE::Transform & xform){
    sinput >> xform.trans[0];
    sinput >> xform.trans[1];
    sinput >> xform.trans[2];
    sinput >> xform.rot[0];
    sinput >> xform.rot[1];
    sinput >> xform.rot[2];
}

void parseMatToTransform( std::istream & sinput, chomp::Transform & xform ){

    chomp::Transform::mat4 mat;

    for ( int i = 0; i < 3; i ++ ){
        for ( int j = 0; j < 4; j ++ ){
            if ( sinput.eof() ){
                RAVELOG_ERROR("Not Enough Values in parseMatToTransform()");
                throw OpenRAVE::openrave_exception("Bad arguments!");
            }
            sinput >> mat( i, j );
        }
    }

    mat(3,3) = 1;

    xform.setFromMatrix( mat );
}
ORTSRConstraint * mod::parseTSR( std::istream & sinput ){

    chomp::Transform T0_w, Tw_e;
    chomp::MatX bounds(6,2);

    int manip_index;
    sinput >> manip_index;

    std::string body_name, link_name;

    sinput >> body_name;
    if (body_name != "NULL"){ sinput >> link_name; }
    else { link_name = "NULL"; }

    parseMatToTransform( sinput, T0_w );
    parseMatToTransform( sinput, Tw_e );

    //parse the bounds of the tsr
    sinput >> bounds( 0, 0); sinput >> bounds( 0, 1);
    sinput >> bounds( 1, 0); sinput >> bounds( 1, 1);
    sinput >> bounds( 2, 0); sinput >> bounds( 2, 1);
    sinput >> bounds( 3, 0); sinput >> bounds( 3, 1);
    sinput >> bounds( 4, 0); sinput >> bounds( 4, 1);
    sinput >> bounds( 5, 0); sinput >> bounds( 5, 1);
    
    //TODO Make this less hacky.
    for ( int i = 0; i < 3; i ++ ){
        if ( bounds( i, 1) >= 1e10 ){
            bounds( i, 1 ) = HUGE_VAL;
        }
        if ( bounds( i, 0) <= -1e10 ){
            bounds( i, 0 ) = -HUGE_VAL;
        }
    }

    return new ORTSRConstraint( this, manip_index, T0_w, bounds, Tw_e,
                                body_name, link_name );
}
    
void mod::parseRobot( std::string & robot_name ){
    if (robot.get()) { 
        if (robot->GetName() != robot_name ) {
            throw OpenRAVE::openrave_exception(
                "Only one robot can be passed!");
        }
    }else {
        robot = environment->GetRobot( robot_name.c_str() );

        if ( !robot.get() ){              
            throw OpenRAVE::openrave_exception(
                    "Failed to get robot");
        }
    }

    active_indices = robot->GetActiveDOFIndices();
    robot->GetActiveDOFLimits( lowerJointLimits, upperJointLimits);
    n_dof = active_indices.size();
 
    if (!robot.get()) {
        throw OpenRAVE::openrave_exception(
                "Robot name not valid");
    }
}


void mod::parsePoint( std::istream& sinput, chomp::MatX & point ){
    if ( active_indices.size() <= 0 ){
        RAVELOG_ERROR("n_dof must be set before states can be added" );
        throw OpenRAVE::openrave_exception("Bad arguments!");
    }

    point.resize( 1, active_indices.size() );
    for ( size_t i = 0; i <active_indices.size() ; i ++ ){
        if (sinput.eof() ){
            RAVELOG_ERROR(
                "Not enough values in input stream for parsePoint" );
            throw OpenRAVE::openrave_exception("Bad arguments!");
        }else {
            sinput >> point(i);
        }
    }   
}

void mod::parseError( std::istream& sinput ){

    std::string cmd;

    while ( !sinput.eof() ){
        RAVELOG_ERROR("argument %s not known!\n", cmd.c_str() );
        sinput >> cmd;
    }
    throw OpenRAVE::openrave_exception("Bad arguments!");
}


void mod::parseCreate(std::ostream & sout, std::istream& sinput)
{

    std::string cmd;
    /* parse command line arguments */
    while (!sinput.eof () ){
        sinput >> cmd;
        
        RAVELOG_INFO( "Executing command: %s\n" , cmd.c_str() );
      
        if (!sinput){ break; }
        if (cmd == "loadrobot"){
            std::string robot_location;
            sinput >> robot_location;
            debugStream << "Location: " << robot_location << std::endl;
            if (!environment.get() ){
                RAVELOG_ERROR( "There is no Environment" );
            }
            environment->Load( robot_location.c_str() );
            RAVELOG_INFO( "Done loading robot" );
        }

        else if (cmd == "robot")
        {
            sinput >> robot_name;
            parseRobot( robot_name );

        }else if ( cmd == "activemanipindex" ){
            int index;
            sinput >> index;

            if( robot.get() )
            {
                active_manip = robot->GetManipulators()[index];
                robot->SetActiveManipulator( active_manip );
                robot->SetActiveDOFs( active_manip->GetArmIndices() );
                active_indices = robot->GetActiveDOFIndices();
                robot->GetActiveDOFLimits(lowerJointLimits,
                                          upperJointLimits);
                n_dof = active_indices.size();
            }
            else{ 
                throw OpenRAVE::openrave_exception(
                        "Robot name not valid");
            }
        }else if ( cmd == "activemanipname" ){
            std::string name;
            sinput >> name;

            if( robot.get() )
            {
              active_manip = robot->SetActiveManipulator( name );
              robot->SetActiveDOFs( active_manip->GetArmIndices() );
              active_indices = robot->GetActiveDOFIndices();
              robot->GetActiveDOFLimits(lowerJointLimits, upperJointLimits);
              n_dof = active_indices.size();
          }
          else{ 
                throw OpenRAVE::openrave_exception(
                        "Robot name not valid");
          }
        }else if (cmd == "startik" ){
            OpenRAVE::Transform xform;
            parseTransform( sinput, xform );
            
            std::vector< OpenRAVE::dReal > solution;
            getIK( xform, solution );
            
            vectorToMat( solution, q0 );
        }else if (cmd == "endik" ){
            OpenRAVE::Transform xform;
            parseTransform( sinput, xform );
            
            std::vector< OpenRAVE::dReal > solution;
            getIK( xform, solution );
            
            vectorToMat( solution, q1 );

        }
        else if ( cmd == "randomstart" ){
            getRandomState( q0 );
            debugStream << "\t\tRandom Start: " << q0 << std::endl;
        }else if ( cmd == "randomend" ){
            getRandomState( q1 );
            debugStream << "\t\tRandom end: " << q1 << std::endl;
        }
        else if (cmd == "q0" ){ parsePoint( sinput, q0 ); }
        else if (cmd == "q1" ||
                 cmd == "adofgoal" ){ parsePoint( sinput, q1); }
        else if (cmd == "jointpadding"){ sinput >> info.jointPadding; }
        else if (cmd == "epsilon"){ sinput >> info.epsilon;}
        else if (cmd == "epsilon_self"){ sinput >> info.epsilon_self; }
        else if (cmd == "obs_factor"){ sinput >> info.obs_factor; }
        else if (cmd == "obs_factor_self"){sinput >> info.obs_factor_self;}
        else if (cmd == "doobserve"){ info.doObserve = true;  }
        else if (cmd == "noobserve"){ info.doObserve = false;  }
        else if (cmd == "nofactory"){ info.noFactory = true;  }
        else if (cmd == "nocollider"){ info.noCollider = true;  }
        else if (cmd == "noselfcollision"){ info.noSelfCollision = true;  }
        else if (cmd == "noenvironmentalcollision"){
            info.noEnvironmentalCollision = true; 
        }   
        // These are unimplemented:
        else if (cmd == "starttraj" ){
            RAVELOG_ERROR( "Starttraj has not been implemented" );
        }
        //these are depracated commands and may or may not be used for
        //  backwards compatibility.
        else if (cmd =="n_points") { sinput >> info.n_max; }
        else if (cmd == "seed" ){ sinput >> info.seed; }
        else if (cmd == "use_hmc"){ info.use_hmc = true;}
        else if (cmd == "hmc_resample_lambda"){ sinput >> info.hmc_lambda;}
        else if (cmd == "lambda" ){ 
            sinput >> info.alpha;
            info.alpha = 1.0/info.alpha;
        }else if (cmd == "use_momentum" ){
            info.use_momentum = true;
        }
        else if (cmd == "do_reject"){
            info.do_not_reject = false;
        }
        //error case
        else{ parseError( sinput ); }
    }
    
    if (size_t( q0.cols() )!= active_indices.size() ){
        std::vector< OpenRAVE::dReal > values;
        robot->GetDOFValues( values, active_indices );
        vectorToMat( values, q0 );
    }
}

void mod::parseViewSpheres(std::ostream & sout, std::istream& sinput)
{

    std::string cmd;
    /* parse command line arguments */
    while (!sinput.eof () ){
        sinput >> cmd;
      
        RAVELOG_INFO( "Executing command: %s\n" , cmd.c_str() );
        if (!sinput){ break; }
        if (cmd == "robot")
        {
            sinput >> robot_name;
            parseRobot( robot_name );
        }

        //error case
        else{ parseError( sinput ); }
    }

}

void mod::parseIterate(std::ostream & sout, std::istream& sinput)
{
    std::string cmd;
    /* parse command line arguments */
    while (!sinput.eof() )
    {
        sinput >> cmd;
        RAVELOG_INFO( "Executing command: %s\n" , cmd.c_str() );

        if (!sinput){ break; }

        if (cmd =="obstol") {
            sinput >> info.obstol;
        }else if (cmd =="n" || cmd == "n_points") {
            sinput >> info.n;
        }else if (cmd =="n_max"){
            sinput >> info.n_max;
        }else if (cmd =="alpha") {
            sinput >> info.alpha;
        }else if ( cmd == "gamma"){
          sinput >> info.gamma;
        }else if (cmd =="min_global_iter") {
            sinput >> info.min_global_iter;
        }else if (cmd =="min_local_iter") {
            sinput >> info.min_local_iter;
        }else if (cmd =="max_global_iter") {
            sinput >> info.max_global_iter;
        }else if (cmd =="n_iter" ) {
            sinput >> info.max_global_iter;
            info.max_local_iter = info.max_global_iter;
        }else if (cmd =="max_local_iter") {
            sinput >> info.max_local_iter;
        }else if (cmd == "timeout" ||
                  cmd == "max_time"){
            sinput >> info.timeout_seconds;
        }
        else if ( cmd == "dolocal"  ){ info.doLocal   = true;  }
        else if ( cmd == "nolocal"  ){ info.doLocal   = false; }
        else if ( cmd == "doglobal" ){ info.doGlobal  = true;  } 
     
        //error case
        else{ parseError( sinput ); }

    }        

    if ( info.n > info.n_max ){
        info.n_max = info.n;
    }

}

void mod::parseGetTraj(std::ostream & sout, std::istream& sinput)
{

    std::string cmd;
    /* parse command line arguments */
    while (!sinput.eof () ){
        sinput >> cmd;
        
        if (!sinput){ break; }
        if (cmd == "no_collision_check"){ 
            info.no_collision_check = true;
        }else if (cmd == "no_collision_exception"){
            info.no_collision_exception = true;
        }else if (cmd == "no_collision_details"){
            info.no_collision_details = true; 
        }
    }
}

void mod::parseDestroy(std::ostream & sout, std::istream& sinput)
{
}
void mod::parseComputeDistanceField(std::ostream & sout, std::istream& sinput)
{
    
    
    double aabb_padding( -1), cube_extent( -1);
    OpenRAVE::KinBodyPtr kinbody;

    bool getall = false;

    std::string cmd, cache_filename("NULL");

    /* parse command line arguments */
    while (!sinput.eof () ){
        sinput >> cmd;
        debugStream << "\t-ExecutingCommand: " << cmd << std::endl;

        if ( cmd == "kinbody" ){
            std::string name;
            sinput >> name;
          
            if (kinbody.get()){
                throw OpenRAVE::openrave_exception(
                    "Only one kinbody can be passed!");
            }
            
            //get the kinbody
            kinbody = environment->GetKinBody(name.c_str());
            
            if (!kinbody.get()){
                std::string error = 
                        "Could not find kinbody named: " + name;
                throw OpenRAVE::openrave_exception( error );
            }
        } else if (cmd == "robot"){
            sinput >> robot_name;
            parseRobot( robot_name );
        }else if ( cmd == "getall" ){
            getall = true;
        }else if (cmd == "aabb_padding"){
            sinput >> aabb_padding;
        }else if (cmd == "cube_extent"){
            sinput >> cube_extent;
        }else if (cmd == "cache_filename"){
            sinput >> cache_filename;
        }
        
        //handle bad arguments
        else{
            while ( !sinput.eof() ){
                std::string argument;
                sinput >> argument;
                RAVELOG_ERROR("argument %s not known!\n", argument.c_str());
                throw OpenRAVE::openrave_exception("Bad arguments!");
            }
        }
   }
   
   if ( !getall && kinbody.get() ){
        //create new sdf object, and put it at the back of the sdfs 
        //  object.
        sdfs.resize( sdfs.size() + 1 );
        DistanceField & current_sdf = sdfs.back();
        if ( aabb_padding >= 0 ){ current_sdf.aabb_padding = aabb_padding; }
        if ( cube_extent >= 0 ){ current_sdf.cube_extent = cube_extent; }
        current_sdf.kinbody = kinbody;
 
        RAVELOG_INFO("Using kinbody %s.\n",
                        current_sdf.kinbody->GetName().c_str());
        RAVELOG_INFO("Using aabb_padding |%f|.\n",
                        current_sdf.aabb_padding);
        RAVELOG_INFO("Using cube_extent |%f|.\n",
                        current_sdf.cube_extent);
        RAVELOG_INFO("Using cache_filename |%s|.\n",
                        cache_filename.c_str());
        //create the sdf.
        current_sdf.createField( environment, cache_filename.c_str() );
   }
   else if ( !getall ){
        throw OpenRAVE::openrave_exception(
                "Need a kinbody to compute a distance field!");
   }else if( getall ){
        std::vector< OpenRAVE::KinBodyPtr > bodies;
        environment->GetBodies( bodies );
        for ( size_t i = 0; i < bodies.size (); i ++){
            
            //if the body is the robot, skip it because we do not want
            //  to calculate collisions for our self.
            if ( bodies[i].get() == robot.get() ){ continue; }

            //create a new sdf object and fill it with the input data.
            sdfs.resize( sdfs.size() + 1 );

            DistanceField & current_sdf = sdfs.back();
            if ( aabb_padding >= 0 ){ 
                current_sdf.aabb_padding = aabb_padding;
            }
            if ( cube_extent >= 0 ){ 
                current_sdf.cube_extent = cube_extent;
            }
            
            //
            current_sdf.kinbody = bodies[i];
            
            RAVELOG_INFO("Using kinbody %s.\n",
                          current_sdf.kinbody->GetName().c_str());
            RAVELOG_INFO("Using aabb_padding |%f|.\n",
                          current_sdf.aabb_padding);
            RAVELOG_INFO("Using cube_extent |%f|.\n",
                          current_sdf.cube_extent);
            RAVELOG_INFO("Using cache_filename |%s|.\n",
                                cache_filename.c_str());

            current_sdf.createField( environment );
        }
    } 
}

void mod::parseAddFieldFromObsArray(std::ostream & sout, std::istream& sinput)
{
}
void mod::parseExecute(std::ostream & sout, std::istream& sinput)
{
}


} /* orchomp namespace */
