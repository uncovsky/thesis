#include "models/env_wrapper.hpp"
#include "models/mdp.hpp"
#include "models/sea_treasure.hpp"
#include "models/resource_gathering.hpp"
#include "geometry/pareto.hpp"
#include "geometry/polygon.hpp"
#include "solvers/brtdp.hpp"
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
    BRTDPSolver brtdp( std::move( env_wrap ), discount_params );

    // solve up to given precision
    auto start_bound = brtdp.solve( precision );

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;
}

void treasure_check(){
    DeepSeaTreasure dst;
    dst.from_file( "../benchmarks/sea_treasure1.txt" );

    TreasureState check;
    check.position = Coordinates(0, 0);
    check.treasure_collected = true;

    for ( const auto &[ k, v ] : dst.get_transition( check, Direction::DOWN ) ){
        std::cout << k << " " << v << "\n";
    }

    

    std::cout << dst.get_reward( check, Direction::DOWN )[0] << std::endl;

    EnvironmentWrapper< TreasureState, Direction, std::vector<double>, double > env_wrap( &dst );

    env_wrap.set_discount_params( { 0.9, 0.9 } );

    env_wrap.init_bound( check, Direction::DOWN );
    std::cout << env_wrap.get_state_action_bound( check, Direction::DOWN );


    BRTDPSolver brtdp( std::move( env_wrap ), { 0.999999, 0.999999 } );

    // solve up to given precision
    auto start_bound = brtdp.solve( 0.1 );

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;
}


void resource_check(){

    ResourceGathering env( 5, 5, // height & width
                           Coordinates( 4, 2 ), // starting pos
                           { Coordinates( 0, 2 ) }, // gold
                           { Coordinates( 1, 4 ) }, // gems 
                           {  } // attackers
                           ); 
    
    ResourceState s( Coordinates( 4, 1 ), { true, true } );
    auto rew = env.get_reward( s, Direction::DOWN );
    std::cout << rew[0] <<  " " << rew[1];
    EnvironmentWrapper< ResourceState, Direction, std::vector<double>, double > env_wrap( &env );

    BRTDPSolver brtdp( std::move( env_wrap ), { 0.9, 0.9 } );

    // solve up to given precision
    auto start_bound = brtdp.solve( 0.1 );

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;
}

int main(){


    /* example: using the function to build the example problematic two state
     * mdp from the CON-MODP paper, each state has two deterministic actions
     * - stay in state / go to other state, 
     *   upon reaching state 0 get 1, 0 reward, upon reaching state 1 get reward 0, 1. */

    /*
    test_brtdp( "../tests/parser_files/model1.tra",  // transition file
                { 
                  "../tests/parser_files/model1.trew",  // reward files
                  "../tests/parser_files/model1_2.trew"
                }, 
                { 0.5, 0.5 }, // discount params
                0,  // id of starting state
                0.1 // precision
                  );

    test_brtdp( "../benchmarks/linked_rings.tra",
                { "../benchmarks/linked_rings.trew" },
                { 0.9, 0.9 },
                0,
                1 );

    treasure_check();

    */

    resource_check();
    test_brtdp( "../benchmarks/resource.tra",
                {
                    "../benchmarks/resource_gold.trew",
                    "../benchmarks/resource_gems.trew",
                },
                { 0.9, 0.9 },
                0,
                0.0000001 );




    return 0;
}
