
#ifndef _TIMER_H_
#define _TIMER_H_

#include <time.h>
#include <vector>
#include <iostream>
#include <boost/unordered_map.hpp>


extern "C"
{
    #include "os.h"
}

class Timer{

  public:
    bool print;
  
  private:
    class single_timer{
        public:
        struct timespec tic, toc, wall_tic, wall_toc;
        double total, elapsed, wall_total, wall_elapsed;
        unsigned int count;
        bool isStopped;
        
        single_timer() : 
                total( 0.0 ), count( 0 ),
                isStopped( true ){}
    };
    typedef std::pair< std::string, single_timer> KeyValuePair;
    typedef boost::unordered_map< std::string, single_timer > Map;

    Map timers; 
    

    single_timer * getTimer( std::string & name, bool post_error=true ){
        
        Map::iterator it = timers.find( name );

        //if the object exists, return the thing.
        if ( it != timers.end() ){ return &(it->second); }
        else if ( post_error ){
            std::cout << "No timer exists by the name: " 
                      << name << "\n";
            return NULL;
        }

        //if the timer does not exist, create and return it.
        KeyValuePair new_timer;
        new_timer.first = name;

        std::pair< Map::iterator, bool> inserted_element = 
                       timers.insert( new_timer );

        return &(inserted_element.first->second);
    }


  public:
    
    Timer() : print(false){}

    void coutElapsed( std::string name ){
        single_timer * t = getTimer(name);
        if ( t ){
            std::cout << "Timer [" << name << "] elapsed time: "
                      << t->elapsed << "s\n";
        }
    }

    void coutTotal( std::string name ){
        single_timer * t = getTimer(name);
        if ( t ){
             std::cout << "Timer [" << name << "] total time: "
                      << t->total << "s\n";
        }
    }


    void start( std::string name ){

        single_timer * t = getTimer(name, false);
        if ( print ){
            std::cout << "Starting " << name << std::endl;
        }

        t->isStopped = false;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &(t->tic));
        clock_gettime(CLOCK_MONOTONIC, &(t->wall_tic));
    }
    double stop( std::string name ){
        
        single_timer * t = getTimer(name);
        if ( !t) {return 0.0;}

        if ( print ){
            std::cout << "Stopping " << name << std::endl;
        }

        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &(t->toc));
        CD_OS_TIMESPEC_SUB(&(t->toc), &(t->tic));

        clock_gettime(CLOCK_MONOTONIC, &(t->wall_toc));
        CD_OS_TIMESPEC_SUB(&(t->wall_toc), &(t->wall_tic));

        t->elapsed = CD_OS_TIMESPEC_DOUBLE(&(t->toc));
        t->wall_elapsed =CD_OS_TIMESPEC_DOUBLE(&(t->wall_toc));
        t->total += t->elapsed;
        t->wall_total += t->wall_elapsed;

        t->count ++;
        t->isStopped = true;

        return t->elapsed;
    }

    double reset( std::string name){
        single_timer * t = getTimer(name);
        if ( !t ) {return 0.0;}

        double temp = t->total;
        t->total = 0;
        t->count = 0;
        t->elapsed = 0;
        return temp;
    }

    double getTotal( std::string name ){
        single_timer * t = getTimer(name);
        if ( !t ) {return 0.0;}
        return t->total;
    }
    double getWallTotal( std::string name ){
        single_timer * t = getTimer(name);
        if ( !t) {return 0.0;}
        return t->wall_total;
    }
    double getWallElapsed( std::string name ){
        single_timer * t = getTimer(name);
        if ( !t ) {return 0.0;} 
        return t->wall_elapsed;
    }
    double getElapsed( std::string name ){
        single_timer * t = getTimer(name);
        if ( !t ) {return 0.0;}
        return t->elapsed;
    }

    //returns true if the timer is already in a start state, elsewise,
    //  it is started.
    bool tryStart( std::string name ){
        single_timer * t = getTimer(name, false);

        if ( t->isStopped ){
            t->isStopped = false;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &(t->tic));
            return false;
        }

        return false;
    }

    //returns the elapsed time if if the timer is already in a
    //  stopped state, elsewise, it is started.
    bool tryStop( std::string name ){
        single_timer * t = getTimer(name);

        if ( !t ) {return false;}
        if ( t->isStopped ){
            return true;
        }

        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &(t->toc));
        CD_OS_TIMESPEC_SUB(&(t->toc), &(t->tic));

        t->elapsed = CD_OS_TIMESPEC_DOUBLE(&(t->toc));
        t->total += t->elapsed;
        t->count ++;
        t->isStopped = true;

        return false;
    }

    unsigned int getCount( std::string name ){
        single_timer * t = getTimer(name);
        if ( !t ) {return 0;}

        return t->count;
    }

    void wait( double time ){
        //wait for given amount of time
        struct timespec ticks_tic;
        struct timespec ticks_toc;

        /* start timing voxel grid computation */
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ticks_tic);

        while ( true ){
          /* stop timing voxel grid computation */
          clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ticks_toc);
          CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
          if ( time < CD_OS_TIMESPEC_DOUBLE(&ticks_toc) ){ break ; }
        }

    }
};

#endif
