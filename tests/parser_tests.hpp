# pragma once

# include <cassert>
# include <iostream>
# include "parser.hpp"


void model1test() {
   PrismParser parser; 

   parser.parse_transition_file( "../tests/parser_files/model1.tra" );
   parser.parse_reward_file( "../tests/parser_files/model1.trew" );
   parser.parse_reward_file( "../tests/parser_files/model1_2.trew" );

   MDP< double > mdp = parser.build_model( 0 );

   assert( mdp.get_current_state() == 0 );
   assert( mdp.get_actions( 0 ) == std::vector< size_t > ( { 0, 1 } ) );
   assert( mdp.get_actions( 1 ) == std::vector< size_t > ( { 0, 1 } ) );
   mdp.step( 1 );
   assert( mdp.get_current_state() == 1 );

   assert( mdp.get_reward( 0, 0 ) == std::vector< double > ( { 1.0, 0.0 } ) );
   assert( mdp.get_reward( 1, 0 ) == std::vector< double > ( { 1.0, 0.0 } ) );
   assert( mdp.get_reward( 0, 1 ) == std::vector< double > ( { 0.0, 1.0 } ) );
   assert( mdp.get_reward( 1, 1 ) == std::vector< double > ( { 0.0, 1.0 } ) );

   
}


void test_parser(){
    model1test();
}
