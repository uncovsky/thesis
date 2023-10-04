#include "benchmarks/frozen_lake.hpp"
#include "benchmarks/sea_treasure.hpp"
#include "benchmarks/racetrack.hpp"
#include "benchmarks/resource_gathering.hpp"

#include "geometry/pareto.hpp"
#include "geometry/polygon.hpp"

#include "models/env_wrapper.hpp"
#include "models/mdp.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

#include "utils/prng.hpp"

#include "parser.hpp"
#include <iostream>




/* runs the algorithm on mdp given by a transition file and 1 or 2 reward files
 * the formats are described here: 
        * https://www.prismmodelchecker.org/manual/Appendices/ExplicitModelFiles
        * the only difference is that the parser simply ignores the first line
        * that is not a comment, in the original prism format this line is used
        * to pass counts of states and actions, but i didn't use it ( hence in
        * the testing files some files simply have a header text at that place )
        *
 * the underlying brtdp solver is called with the passed discount parameters 
 * and samples trajectories from the given starting state of the mdp
 * precision gives the stopping condition, i.e how close w.r.t to hausdorff
 * distance the difference of the starting bounds has to be before we terminate
 * the computation
 *
 * both pareto curves of the starting state are then output to text files
 * starting_lower.txt, starting_upper.txt in the same directory as ( both their
 * vertices and facets respectivelly )
 */

void test_brtdp( const std::string &transition_file,
                 const std::vector< std::string > &reward_files,
                 std::vector< double > discount_params,
                 size_t starting_state=0,
                 double precision=0.1 ) {

    // construct parser object
    PrismParser parser;

    // parse the provided files
    auto mdp = parser.parse_model( transition_file,
                                   reward_files,
                                   starting_state );

    // construct a wrapper object around the mdp ( initializes the object
    // bounds, and the solver uses it to interact with the model )
    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap( &mdp );

    // construct the solver object using the wrapper and discount parameters
    ExplorationSettings< double > basic;
    basic.discount_params = discount_params;
    basic.precision = precision;

    BRTDPSolver brtdp( std::move( env_wrap ), basic );

    // solve up to given precision
    auto start_bound = brtdp.solve();

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;
}

void treasure_check(){
    DeepSeaTreasure dst, dst_convex;
    dst.from_file( "../benchmarks/sea_treasure1.txt" );
    dst_convex.from_file( "../benchmarks/sea_treasure_convex.txt" );

    EnvironmentWrapper< TreasureState, Direction, std::vector<double>, double > env_wrap( &dst );

    ExplorationSettings< double > config;
    config.discount_params = { 0.99, 0.99 };
    BRTDPSolver brtdp( std::move( env_wrap ), config );

    auto start_bound = brtdp.solve();
    std::cout << start_bound;

    brtdp.load_environment( dst_convex );
    start_bound = brtdp.solve();
    std::cout << start_bound;
}


void resource_check(){

    ResourceGathering env( 5, 5, // height & width
                           Coordinates( 4, 2 ), // starting pos
                           { Coordinates( 0, 2 ) }, // gold
                           { Coordinates( 1, 4 ) }, // gems 
                           {  } // attackers
                           ); 
    
    EnvironmentWrapper< ResourceState, Direction, std::vector<double>, double > env_wrap( &env );
    EnvironmentWrapper< ResourceState, Direction, std::vector<double>, double> env_wrap2( &env );

    ExplorationSettings< double > config;
    BRTDPSolver brtdp( std::move( env_wrap ), config );

     CHVIExactSolver chvi( std::move( env_wrap2 ), config );
     auto start_bound2 = chvi.solve();
     std::cout << start_bound2;

    // solve up to given precision
    auto start_bound = brtdp.solve();

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;

}

int main(){


    PRNG gen;
    std::set< Coordinates > randpits;
    for ( int i = 0; i < 25; ++i ) {
       randpits.insert( Coordinates( gen.rand_int( 1, 25), gen.rand_int ( 1, 25 ) ) );
    }

    for ( auto &x : randpits ) {
        std::cout << x << "\n";
    }


    FrozenLake lake, lake2( 25, 25, randpits, 0.3 );

    EnvironmentWrapper< Coordinates, Direction, std::vector< double >, double > envw( &lake );
    EnvironmentWrapper< Coordinates, Direction, std::vector< double >, double > envw2( &lake2 );

    ExplorationSettings< double > config;
    config.discount_params =  { 0.99, 0.99 };
    config.max_trajectory = 1000;
    config.precision = 0.1;
    config.filename = "laketest";
    config.lower_bound_init = { 0, -1000 };
    config.upper_bound_init = { 1, 1 };

    BRTDPSolver brtdp( std::move( envw2 ), config );
    brtdp.solve();
    return 0; 

    CHVIExactSolver chvi( std::move( envw2 ), config );
    auto res = chvi.solve();

    std::cout << res;

    // auto res = brtdp.solve( 0.1 );

    return 0;
}
