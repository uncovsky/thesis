#pragma once

#include <memory>
#include <sstream>
#include <algorithm>
#include "models/environment.hpp"
#include "utils/eigen_types.hpp"

/* records statistics about given state 
 * record_obj_t is the type of record object kept for every state action pair
*  in our case the pareto curve object, but can be anything of interest for the
*  solver */

template < typename action_t, typename record_obj_t  > 
class StateRecord{
    
    using record_obj_ptr = std::unique_ptr < record_obj_t >;
    using record_obj_raw_ptr = record_obj_t *;
    

    // state action records (S x A)
    std::map< action_t, record_obj_ptr > q_records; 
    std::map< action_t, size_t > visit_counts;
    
    size_t visit_count;

public:
    StateRecord() : 
                    q_records()    , 
                    visit_counts() ,
                    visit_count(0) {}

    StateRecord(action_t action) : 
                    q_records()    , 
                    visit_counts() ,
                    visit_count(0) {
                                       init_record(action);
                                        visit(action);
                                    }

    size_t get_visit_count() const{
        return visit_count;
    }
    
    size_t get_visit_count(action_t action) const{
        return visit_counts[action];
    }
    
    void visit(action_t action){
        visit_count++;
        visit_counts[action]++;
    }

    void init_record(action_t action){
        q_records.emplace( action, std::make_unique< record_obj_t >() );
    }
    
    record_obj_raw_ptr get_record_ptr(action_t action){
        return q_records[action].get();
    }

};

/* this class is used to interact with the underlying environment, recording
 * statistics, simulation, logging, etc. */

template < typename state_t, typename action_t, typename reward_t , typename record_obj_t >
class EnvironmentWrapper{
    
    using record_t = StateRecord < action_t, record_obj_t >;
    using record_ptr = std::unique_ptr< record_t >;
    using record_raw_ptr = record_t *;

    Environment< state_t, action_t, reward_t > *env;
    std::map < state_t, record_ptr > records;

public:

    EnvironmentWrapper() : env( nullptr ), records() {}
    EnvironmentWrapper( Environment< state_t, action_t, reward_t > *env ) : env( env ), records() {}

    using Observation = typename Environment< state_t, action_t, reward_t > :: Observation;

    state_t get_current_state() const {
        return env->get_current_state();
    }

    std::vector< action_t > get_actions() const {
        return env->get_actions( get_current_state() );
    }

    std::map< state_t, double > get_transition(state_t state, action_t action) const {
        return env->get_transition( state, action );
    }

    std::vector< action_t > get_actions( state_t state ) const {
        return env->get_actions( state );
    }

    reward_t get_expected_reward( state_t s, action_t a ) {
        return env->get_reward( s, a );
    }

    reward_t get_expected_reward( state_t s, action_t a, state_t succ_s ) {
        return env->get_reward( s, a );
    }

    void clear_records(){
        records.clear();
    }
    
    Observation reset( unsigned seed=0, bool reset_records=true ) {
        if (reset_records) { clear_records(); }
        return env->reset( seed );
    }

    Observation step( action_t action ) {
        state_t current_state = get_current_state();
        auto [ next_state, reward, terminated ] = env->step( action );

        // if the record is found, modify it
        if ( auto it = records.find( current_state ); it == records.end() ) {
            if ( it->get_visit_count( action ) == 0 ) { it->init_record( action ); }
            it->visit( action );
        }
        
        else{
            record_t new_record( action );
            records.emplace( current_state, std::move( new_record ) );
        }
        
        return { next_state , reward, terminated };
    }

    record_ptr get_record( state_t state, action_t action ) {
         if ( auto it = records.find[state]; it != records.end() ) {
             return it->get_record_ptr(action);
         }
         return nullptr;
    }
};


