
#ifndef _ORCHOMP_COLLISION_PRUNER_H_
#define _ORCHOMP_COLLISION_PRUNER_H_

#include <openrave/openrave.h>
#include "orchomp_distancefield.h"
#include "orchomp_kdata.h"
#include <set>

namespace orchomp{

class SphereCollisionHelper;

typedef std::vector<std::pair<int,int> > CollisionReport;
/*
class IntervalBase {
    getOverlaps( PruningIterator & pruner,
                 std::stack<pair<int, int> > & overlaps );
    
}
*/

class Interval{
  public:
    size_t index;
    bool isOpen;
    Interval *next, *prev;
    Interval() : isOpen( false ){}

    bool tryClose(){
        if ( isOpen ){ 
            if ( next ){ next->prev = prev; }
            if ( prev ){ prev->next = next; }
            isOpen = false;
            return true;
        }
        return false;
    }
    void open( Interval * head ){
        isOpen = true;

        //set the pointers of this object
        this->prev = head;
        this->next = head->next;

        //set the pointers of this's neighbors
        if( head->next ){ head->next->prev = this; }
        head->next = this;
    }
};


class CollisionPruner{

  public:

    class Node{
      public:
        Node *next, *prev;
        double value;
        size_t index;
    };

  //private member variables
  private: 
    const int axis;
    const std::vector< Sphere > & spheres;
    const std::vector< DistanceField> & sdfs;
  
    Node * head;
    Interval * head_interval;
    std::vector< std::pair<Node, Node> > nodes;
    std::vector< Interval > intervals;

  
  //public member functions
  public:
    void sort( const std::vector< OpenRAVE::Vector > & positions, 
               size_t n_set_positions);
    CollisionPruner( int axis, 
                     const std::vector< Sphere > & spheres,
                     const std::vector< DistanceField> & sdfs); 
    void getPotentialCollisions( std::vector<
                                 std::pair<size_t,size_t> > & coll_pairs );
    bool checkPotentialCollisions( SphereCollisionHelper * checker );

  private:

    void closeOpenIntervals( Interval * head_interval );
    void printNodeList( Node * start, Node * end = NULL );
    void printIntervalList( Interval * start );
    void initNodes();
    void initIntervals();
    virtual void assertSorted(Node * end);
    virtual void assertSorted();
    void insertion_sort();    
    void insert( Node * list_tail, Node * current_element );

};


class ArrayInterval{
  public:
    int * open_indices;
    int size;
  private:
    int ** open_intervals;

  //member functions
  public:
    void init( int n_objects ){
        size = 0;
        open_indices = new int[ n_objects ];
        open_intervals = new int*[ n_objects ];

        //set all the intervals to closed.
        for ( int i = 0; i < n_objects; i ++ ){
            open_intervals[i] = NULL;
        }
    }
    ~ArrayInterval(){
        if (open_indices ){ delete open_indices; }
        if (open_intervals){ delete open_intervals; }
    }

    inline bool tryClose( int index ){
        if ( open_intervals[index] == NULL ){
            return false;
        }
        size --;
        if ( size > 0 ){
            //move the back of the list to be at the 
            //  now vacant position
            int * location = open_intervals[index];
            *location = open_indices[size];
            open_intervals[ open_indices[size] ] = location;
        }
        
        //Invalidate the pointer to close the interval;
        open_intervals[index] = NULL;
        return true;
    }
    inline void open( int index ){
        int * location = &( open_indices[size] );
        *location = index;
        open_intervals[index] = location;
        size++;
    }
    inline void closeAll(){
        for ( int i = 0; i < size; i ++ ){
            open_intervals[ open_indices[i] ] = NULL;
        }
        size = 0;
    }
};


class ArrayCollisionPruner{
    
  private:

    int n_active;
    const int axis;
    const std::vector< Sphere > & spheres;
    const std::vector< DistanceField> & sdfs;

    //first is the value
    //second is the index.
    typedef std::pair< double, size_t> ArrayNode;
    std::vector< std::pair< ArrayNode, ArrayNode > > nodes;
    std::vector< ArrayNode * > sorted_nodes;
    
    std::vector< Interval > intervals;

  public:
    ArrayCollisionPruner( int axis, 
                          const std::vector< Sphere > & spheres,
                          const std::vector< DistanceField> & sdfs);
    virtual void sort( const std::vector< OpenRAVE::Vector > & positions, 
                       size_t n_set_positions);
    bool checkPotentialCollisions( SphereCollisionHelper * checker );
    void getPotentialCollisions( CollisionReport & collisions);

  private:
    void assertSorted();
    void sortArray();

    void check( CollisionReport & collisions );

    void initIntervals();
    void addCollisions( CollisionReport & sphere_on_sphere,
                             int index, Interval* current );

};

}//namespace

#endif
