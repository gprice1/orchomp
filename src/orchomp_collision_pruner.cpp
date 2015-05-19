#include "orchomp_collision_pruner.h"
#include "orchomp_collision.h"

namespace orchomp{

    CollisionPruner::CollisionPruner(
                                  int axis, 
                                  const std::vector< Sphere > & spheres ,
                                  const std::vector< DistanceField> & sdfs):
        axis( axis ), spheres( spheres ), sdfs( sdfs )
    {
        
        nodes.resize( spheres.size() + sdfs.size() );

        initNodes();
        
        head = &(nodes[0].first);
        
        initIntervals();
    }
    
    void CollisionPruner::initNodes()
    {   
        for ( size_t i = 0; i < nodes.size(); i ++ ){
            nodes[i].first.index = i;
            nodes[i].second.index = i;
            
            //point them at eachother.
            nodes[i].second.prev = &(nodes[i].first);
            nodes[i].first.next =  &(nodes[i].second);

            //set the previous of the first node
            if ( i == 0 ){ nodes[i].first.prev = NULL; }
            else { nodes[i].first.prev = &( nodes[i-1].second ); }
            
            //set the next of the second node.
            if ( i + 1 == nodes.size() ){ nodes[i].second.next = NULL;}
            else {  nodes[i].second.next = &(nodes[i+1].first); }
        }
        
        //set the positions of all of the sdfs.
        for ( size_t i = 0; i < sdfs.size(); i ++ ){
            OpenRAVE::Vector lower, upper;
            sdfs[i].getBounds( lower, upper );
            
            size_t node_index = spheres.size() + i;
            nodes[ node_index ].first.value = lower[axis];
            nodes[ node_index ].second.value = upper[axis];
        }
    }
    
    void CollisionPruner::initIntervals(){

        intervals.resize( spheres.size() + sdfs.size() );
        for( size_t i=0; i < intervals.size(); i ++ ){
            intervals[i].index = i;
        }
    }

    void CollisionPruner::assertSorted( Node * end ){
        Node * current = head;
        int size = 1;
        std::cout << "Values: " << current->value;
        bool broken = false;

        while ( current != end ){
            if ( current->value > current->next->value ){
                broken = true;
            }
            current = current->next;
            std::cout << ", " << current->value;
            size ++;
        }
        std::cout << "\n\tSize: " << size << std::endl;
        assert( !broken );
    }

    void CollisionPruner::assertSorted()
    {
        Node * current = head;
        size_t count = 0;

        assert( head->prev == NULL );

        bool broken = false;
        while ( current->next != NULL ){
            std::cout << "Current: " << current->value
                      << " Next: " << current->next->value
                      << "\n";
            if (current->value > current->next->value ){
                broken = true;
            }

            current = current->next;
            count ++;
        }
        std::cout << "Count: " << count << std::endl;
        assert( count == nodes.size()*2 - 1 );
        assert( !broken );
        
    }

    void CollisionPruner::sort( 
               const std::vector< OpenRAVE::Vector > & positions, 
               size_t n_set_positions)
    {
        assert( positions.size() >= n_set_positions );
        assert( spheres.size() >= n_set_positions );
        
        for ( size_t i = 0; i < n_set_positions; i ++ ){
            const double pos = positions[i][axis];
            const double rad = spheres[i].radius;

            nodes[i].first.value  = pos - rad;
            nodes[i].second.value = pos + rad;
        }

        insertion_sort();
    }

    void CollisionPruner::insertion_sort()
    {

        Node * sorted_tail = head;

        while ( sorted_tail->next != NULL ){

            //if the element is out of place then sort it
            if ( sorted_tail->value > sorted_tail->next->value ){
                
                Node * next_element = sorted_tail->next->next;
                //perform insertion
                insert( sorted_tail, sorted_tail->next);
                sorted_tail->next = next_element;
                sorted_tail->next = next_element;
            }
            //the next element is in the correct place, so
            //  make the tail of the sorted list the next element.
            else { 
                sorted_tail->next->prev = sorted_tail;
                sorted_tail = sorted_tail->next;
            }

        }
    }
        

    void CollisionPruner::insert( Node * list_tail,
                                  Node * current_element )
    {
        
        //if the prev is null, then we have hit the end of the list, so 
        //  we stop.
        const double value = current_element->value;

        while ( list_tail->prev != NULL ){
            list_tail = list_tail->prev;
            
            //we have found the correct spot for the current element,
            //  it should be placed, after list_tail, because 
            //  its value is larger.
            if ( list_tail->value <= value ){

                //set the pointers of the current element.
                current_element->prev = list_tail;
                current_element->next = list_tail->next;
               
                //set the pointers of the elements surrounding 
                //  current_element
                list_tail->next->prev = current_element;
                list_tail->next = current_element;

                return;
            }
        }

        //we have gotten to the end of the list, and the current
        //  element has the smallest value.
        head = current_element;
        current_element->prev = NULL;
        current_element->next = list_tail;
        list_tail->prev = current_element;
    }
    
    void CollisionPruner::getPotentialCollisions( 
                     std::vector< std::pair<size_t,size_t> > & coll_pairs)
    {
        
        Node * current_node = head;
        Interval * head_interval = NULL;
        
        while ( current_node != NULL ){
            const size_t index = current_node->index;

            //Try to close the interval. If false is returned, the closure
            //  failed, so catalogue the collisions, and then open
            //  the interval.
            if ( !intervals[ index ].tryClose() ){
                Interval * curr_interval = head_interval;

                while ( curr_interval != NULL ){
                    coll_pairs.resize( coll_pairs.size() + 1 );
                    coll_pairs.back().first = curr_interval->index;
                    coll_pairs.back().second = index;

                    curr_interval = curr_interval->next;
                }
            }
            current_node = current_node->next;
        }
    }
    
    void CollisionPruner::printNodeList( Node * start, Node * end){
        std::cout << "[";
        while ( start != end ){
            std::cout << start->index << ":" << start->value << ", ";
            start = start->next;
        }
        std::cout << "]\n";
    }

    void CollisionPruner::printIntervalList( Interval * start ){
        std::cout << "[";
        while ( start != NULL ){
            std::cout << start->index << ", ";
            start = start->next;
        }
        std::cout << "]\n";
    }
    
    void CollisionPruner::closeOpenIntervals( Interval * head_interval ){
        while ( head_interval != NULL ){
            head_interval->isOpen = false;
            head_interval = head_interval->next;
        }
    }

    bool CollisionPruner::checkPotentialCollisions( 
                           SphereCollisionHelper * checker )
    {
        
        Node * current_node = head;
        Interval head_interval;
        head_interval.next = NULL;
        
        while ( current_node != NULL ){
            const size_t index = current_node->index;

            //Try to close the interval. If false is returned, the closure
            //  failed, so catalogue the collisions, and then open
            //  the interval.

            if( !intervals[ index ].tryClose() ){
                Interval * curr_interval = head_interval.next;

                while ( curr_interval != NULL ){
                    if (checker->checkCollision( curr_interval->index,
                                                 index ) )
                    {
                        //close all of the open intervals.
                        closeOpenIntervals( head_interval.next );
                        return true;
                    }
                    curr_interval = curr_interval->next;
                }
                intervals[ index ].open( &head_interval );
            }

            current_node = current_node->next;
        }
        return false;
    }



//////////////////////////////////////////////////////////////
//Array List Collision Pruning ///////////////////////////////
//////////////////////////////////////////////////////////////
   ArrayCollisionPruner::ArrayCollisionPruner (
                          int axis, 
                          const std::vector< Sphere > & spheres,
                          const std::vector< DistanceField> & sdfs ) :
        axis(axis), spheres(spheres), sdfs(sdfs)
    {


        initIntervals();
        nodes.resize( sdfs.size() + spheres.size() );
        sorted_nodes.resize( nodes.size() * 2 );
        
        //set the indices of the nodes,
        //  set the sorted list pointers to point at the nodes.
        for ( size_t i = 0; i < nodes.size(); i ++ ){

            //set the indices of the start and endpoints
            nodes[i].first.second = i;
            nodes[i].second.second = i;

            sorted_nodes[i*2  ] = &(nodes[i].first);
            sorted_nodes[i*2+1] = &(nodes[i].second);
        }
            
        //set the positions of all of the sdfs.
        for ( size_t i = 0; i < sdfs.size(); i ++ ){
            OpenRAVE::Vector lower, upper;
            sdfs[i].getBounds( lower, upper );
            
            size_t node_index = spheres.size() + i;
            nodes[ node_index ].first.first = lower[axis];
            nodes[ node_index ].second.first = upper[axis];
        }

    }

    void ArrayCollisionPruner::initIntervals(){
        intervals.resize( spheres.size() + sdfs.size() );
        for( size_t i=0; i < intervals.size(); i ++ ){
            intervals[i].index = i;
        }
    }

    void ArrayCollisionPruner::assertSorted(){
        for ( size_t i = 0; i < sorted_nodes.size()-1; i ++ ){
            assert( sorted_nodes[i]->first <= sorted_nodes[i+1]->first );
        }
    }

    void ArrayCollisionPruner::sort(
              const std::vector< OpenRAVE::Vector > & positions, 
              size_t n_set_positions )
    {
        n_active = n_set_positions;

        for ( size_t i = 0; i < n_set_positions; i ++ ){
            const double pos = positions[i][axis];
            const double rad = spheres[i].radius;

            nodes[i].first.first  = pos - rad;
            nodes[i].second.first = pos + rad;
        }
        
        sortArray();
    }
    
    void ArrayCollisionPruner::sortArray(){

        for ( std::vector<ArrayNode*>::iterator i = sorted_nodes.begin()+1;
              i != sorted_nodes.end();
              ++i )
        {
            for (std::vector<ArrayNode*>::iterator j = i;
                 j > sorted_nodes.begin() && (*j)->first < (*(j-1))->first;
                 j-- )
            {
                std::iter_swap( j, j-1);
            }
        }
    }
    
    inline void ArrayCollisionPruner::addCollisions(
                                    CollisionReport & collisions,
                                    int index1, Interval* current )
    {
        while ( current != NULL ){
            collisions.resize( collisions.size() + 1 );
            collisions.back().first = index1;
            collisions.back().second = current->index;

            current = current->next;
        }
    }

    void ArrayCollisionPruner::getPotentialCollisions(
                    CollisionReport & collisions)
    {
        
        Interval active_head, inactive_head;
        active_head.next = NULL;
        inactive_head.next = NULL;
        
        for ( std::vector<ArrayNode*>::iterator i = sorted_nodes.begin();
              i != sorted_nodes.end();
              ++i )
        {
            const int index = (*i)->second;

            //Try to close the interval. If false is returned, the closure
            //  failed, so catalogue the collisions, and then open
            //  the interval.
            if( !intervals[ index ].tryClose() ){

                addCollisions( collisions, index, active_head.next );

                //if the index is active:
                if (index < n_active){
                    addCollisions( collisions, index,
                                   inactive_head.next );
                    intervals[ index ].open( &active_head );
                }
                else {
                    intervals[ index ].open( &inactive_head );
                }
            }
        }
    }


    bool ArrayCollisionPruner::checkPotentialCollisions( 
                               SphereCollisionHelper * checker )
    {
        
        Interval active_head, inactive_head;
        active_head.next = NULL;
        inactive_head.next = NULL;
        
        for ( std::vector<ArrayNode*>::iterator i = sorted_nodes.begin();
              i != sorted_nodes.end();
              ++i )
        {
            const int index = (*i)->second;

            //Try to close the interval. If false is returned, the closure
            //  failed, so catalogue the collisions, and then open
            //  the interval.
            if( !intervals[ index ].tryClose() ){
                
                Interval * curr_interval = active_head.next;

                while ( curr_interval != NULL ){
                    if ( checker->checkCollision( index,
                                                  curr_interval->index )){
                        return true;
                    }
                    curr_interval = curr_interval->next;
                }

                //if the index is active:
                if (index < n_active){

                    curr_interval = inactive_head.next;
                    while ( curr_interval != NULL ){
                        if ( checker->checkCollision( index,
                                              curr_interval->index )){
                            return true;
                        }
                        curr_interval = curr_interval->next;
                    }

                    intervals[ index ].open( &active_head );
                }
                else { intervals[ index ].open( &inactive_head ); }
            }
        }
        return false;
    }



}//namespace
