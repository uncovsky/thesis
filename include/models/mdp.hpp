#pragma once

#include <vector>
#include "models/environment.hpp"
#include "utils/prng.hpp"
#include "utils/eigen_types.hpp"

 /* basic mdp class utilizing sparse matrices to 
  * store transition and reward function data
  */


template < typename reward_t >
class MDP : public Environment< size_t, size_t , std::vector< reward_t > > {

    typedef std::vector< reward_t > reward_vec ;
    typedef Matrix3D< double > TransitionMatrix;
    typedef Matrix3D< reward_t > RewardMatrix;

    // Observation ( the return value after each step ),
    // includes a vectorial reward, to support multiple objectives 
    using typename Environment< size_t, size_t, reward_vec > :: Observation;

    
    size_t initial_state, current_state;

    // reward bounds for each objective
    std::pair< reward_vec, reward_vec > reward_bounds;

    /* S x A x S - \delta(s,a,s'), transition matrix of probabilities
     * transitions[i] -> transition matrix of given state i (A x S), where A
     * are permitted actions in state i 
     */
    TransitionMatrix transitions;

    /* (A x S) -> reward, vector for each reward model ( each dimension )
     * rows of the respective matrices correspond to action indices
     * for consistency with the transition matrices */

    RewardMatrix reward_models;

    PRNG gen;

public:
    MDP() : transitions() ,
            reward_models() ,
            reward_bounds( { reward_vec(), reward_vec() } ) ,
            initial_state( 0 )  ,
            current_state( 0 ) ,
            gen(){}


    // copy from other and set state to initial
    MDP( const MDP& other ) : transitions( other.get_transition_matrix() ),
                              reward_models( other.get_reward_matrix() ),
                              reward_bounds( other.reward_range() ),
                              initial_state( other.get_initial_state() ),
                              current_state( other.get_initial_state() ),
                              gen(  ){ }


    MDP( const TransitionMatrix& transitions, const RewardMatrix& rewards, 
              std::pair< reward_vec, reward_vec > reward_bounds, size_t s )
            : transitions( transitions ) ,
              reward_models( rewards ) ,
              reward_bounds( reward_bounds ) ,
              initial_state( s ) ,
              current_state( s ) ,
              gen(){}


    // move constructor
    MDP( TransitionMatrix&& transitions, RewardMatrix&& rewards, 
              std::pair< reward_vec, reward_vec > reward_bounds, size_t s )
            : transitions( std::move(transitions) ) ,
              reward_models( std::move(rewards) ) ,
              reward_bounds( reward_bounds ) ,
              initial_state( s ) ,
              current_state( s ) ,
              gen(){}


    std::vector< size_t > get_actions( size_t state ) const override {

        std::vector< size_t > available_actions;
        
        /* actions are saved in rows for easy traversal of successors */
        for ( size_t i = 0; i < transitions[state].outerSize(); ++i ) {

            Matrix2D< double >::InnerIterator it( transitions[state], i );

            // if nonempty row (available action), remember its index
            if (it) { available_actions.push_back(i); }
        }

        return available_actions;
    }

    RewardMatrix get_reward_matrix() const {
        return reward_models;
    }

    TransitionMatrix get_transition_matrix() const {
        return transitions;
    }

    size_t get_initial_state() const {
        return initial_state;
    }


    std::map< size_t, double > get_transition( size_t state, 
                                               size_t action ) const override {

        std::map< size_t, double > delta;

        // iterate over every entry in the row corresponding to delta(s,a)
        for ( Matrix2D< double >::InnerIterator it( transitions[state], action ); it; ++it ) {
            delta.emplace( it.col(), it.value() );
        }

        return delta;
    }



    std::pair< reward_vec, reward_vec > reward_range() const override {
        return reward_bounds;
    }



    reward_vec get_reward( size_t state, size_t action ) override{

        reward_vec rew;
        for ( auto& reward_model : reward_models ) {
                rew.push_back( reward_model.coeffRef( action, state ) );
        }

        return rew;
    }



    size_t get_current_state() const override {
        return current_state;
    }


    bool is_terminal_state( size_t state ) const {
        std::vector< size_t > avail_actions = get_actions( state );
        for ( size_t action : avail_actions ) {

            auto transitions = get_transition( state, action );

            if ( 
                 ( transitions.size() > 1 ) || 
                 ( !approx_equal( transitions[state], 1.0 ) )
               )
                 {
                    return false;
                 }
        }
        
        return true;
    }



    std::string name() const override {
        return "Sparse-MDP";
    }



    Observation step( size_t action ) override {

        reward_vec reward = get_reward( current_state, action );

        // index of successor state in sparse matrix row 
        size_t next_state = gen.sample_distribution( get_transition( current_state, action ) );

        current_state = next_state;

        return { next_state, reward, is_terminal_state( next_state ) };
    }



    Observation reset( unsigned seed ) override{

        // zero seed -> random initialization
        if ( seed == 0 ) {  gen.seed(); }
        else            {  gen.seed( seed ); }
        
        current_state = initial_state;

        reward_vec default_rew( reward_models.size(), reward_t( 0 ) );

        return { initial_state , default_rew, is_terminal_state( initial_state ) };
    }
};



