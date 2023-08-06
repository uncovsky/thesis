# pragma once

# include <cassert>
# include <iostream>
# include "parser.hpp"


void model1test() {
   PrismParser parser; 

   MDP< double > mdp = parser.parse_model( "../tests/parser_files/model1.tra",
                                            { "../tests/parser_files/model1.trew",
                                              "../tests/parser_files/model1_2.trew"},
                                            0 );


   assert( mdp.get_current_state() == 0 );
   assert( mdp.get_actions( 0 ) == std::vector< size_t > ( { 0, 1 } ) );
   assert( mdp.get_actions( 1 ) == std::vector< size_t > ( { 0, 1 } ) );
   mdp.step( 1 );
   assert( mdp.get_current_state() == 1 );

   assert( mdp.get_reward( 0, 0 ) == std::vector< double > ( { 1.0, 0.0 } ) );
   assert( mdp.get_reward( 1, 0 ) == std::vector< double > ( { 1.0, 0.0 } ) );
   assert( mdp.get_reward( 0, 1 ) == std::vector< double > ( { 0.0, 1.0 } ) );
   assert( mdp.get_reward( 1, 1 ) == std::vector< double > ( { 0.0, 1.0 } ) );

   MDP< double > mdp2 = parser.parse_model( "../tests/parser_files/csma.tra",
                                            { "../tests/parser_files/csma.trew", "../tests/parser_files/csma.trew" }, 0 );

    auto [ min, max ] = mdp2.reward_range();

    for ( auto x : min )
        std::cout << x << " ";
    std::cout << "\n\n";
    for ( auto x : max )
        std::cout << x << " ";

    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap3( &mdp2 );
    BRTDPSolver brtdp( std::move( env_wrap3 ) , { 0.9, 0.9 } );
    brtdp.solve( 0.2 );

}


void test_parser(){
    model1test();
}
