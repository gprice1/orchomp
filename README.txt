
Basic Information
=================
Authors: Temple Price, Matt Zucker, Chris Dellin
Emails: gprice1@swarthmore.edu, mzucker1@swarthmore.edu, ??

What: ORChomp 
     - an openrave plugin wrapping an implementation of multigrid-chomp
Where: CMU Robotics Institute and Swarthmore College

Classes
=======

mod - The module class. This brings all the other classes together into
    one giant mess. It takes user input and an OR envronment and plans
    joint trajectories. 
    The code for this class is split into 3 .cpp files: orchomp_mod.cpp, 
    orchomp_mod_parse.cpp, orchomp_mod_utils.cpp . The first of these
    holds the implementation for all of the major functionality, mainly
    the functions that the user can call. orchomp_mod_parse.cpp holds
    most of the functions for parsing user input. orchomp_mod_utils.cpp
    contains many convenience functions such as converting between
    matrices and vectors, or small visualization functions.
    Important Files: orchomp_mod.h
                     orchomp_mod.cpp
                     orchomp_mod_parse.cpp
                     orchomp_mod_utils.cpp

SphereCollisionHelper - this class handles all of the collisions. It uses
    spheres and distance fields to represent the collideable objects in the
    world. It is used by chomp to get gradients and costs for the collision
    term of the objective function. This inherits from the class 
    ChompGradientHelper. 
    Important Files: orchomp_collision.h
                     orchomp_collision.cpp
                     orchomp_collision_pruner.h
                     orchomp_collision_pruner.cpp
                     chomp-multigrid/chomp/ChompGradient.h
                     chomp-multigrid/chomp/ChompGradient.cpp
 
Pruner - This is used by SphereCollisionHelper to reduce the number of
    pairwise collision checks. It implements a broad sphase collision
    detection method called sort and sweep. It will only detect collisions
    occuring on the 'active spheres' of the robot. The active spheres are
    the spheres that are attached to joints that can move in the current
    planning environment. This is a pretty heavily optimized implementation,
    so there are some problems with code legibility.
    Important Files: orchomp_collision_pruner.h
                     orchomp_collision_pruner.cpp

DistanceField - The distance field object. Used in collision detection.
    It uses an implementation of distance fields from Matt Zucker's common
    code. It also implements flood filling, so hollow meshes, as long
    as there are no holes in the mesh, should be accurately represented as
    solid objects.
    Important Files: orchomp_distancefield.h
                     orchomp_distancefield.cpp
                     chomp-multigrid/mzcommon/DtGrid.h 
                     chomp-multigrid/mzcommon/DtGrid.cpp 

Sphere - This is an object for collision detection. This is a light object
    that is used by the SphereCollisionHelper object.
    Important Files: orchomp_sphere.h

Kdata and Kdata_reader - reads the collision geometry data from the 
    robot xml files. This class is used by the SphereCollisionHelper class
    to load the spheres.
    Important Files: orchomp_kdata.h
                     orchomp_kdata.cpp
                     orchomp_sphere.h

ORConstraintFactory and ORTSRConstraint - The ORConstraintFactory is a 
    wrapper around the chomp-multigrid/chomp/ConstraintFactory. It stores
    constraints, and the interval over which they are active in the 
    trajectory (0,1). The ORTSRConstraint is a wrapper around the
    TSRConstraint, and it implements TSR constraints in openrave. If 
    constrained planning is being done, this class should be passed to
    chomp.
    Important Files: orchomp_constraint.h
                     orchomp_constraint.cpp
                     chomp-multigrid/chomp/Constraint.h
                     chomp-multigrid/chomp/Constraint.cpp
                     chomp-multigrid/chomp/ConstraintFactory.h
                     chomp-multigrid/chomp/ConstraintFactory.cpp

Chomp - the optimizer used by the ORchomp module.
    Important Files: chomp-multigrid/chomp/Chomp.h 
                     chomp-multigrid/chomp/Chomp.cpp 
                     chomp-multigrid/chomp/ChompGradient.h 
                     chomp-multigrid/chomp/ChompGradient.cpp


