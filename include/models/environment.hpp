#pragma once

#include <tuple>

/* 
 *  General environment interface implementing to GymRL API for easy benchmarking
 *      
 */
template < typename state_t, typename action_t, typename reward_t >
class Environment{

    bool simulation;
    state_t current_state, simulation_state;

    using Observation = std::tuple< state_t, reward_t, bool, bool >;
    

public:
    virtual Observation step(const action_t&) = 0;

    // enable potential randomisation of the env
    virtual Observation reset(unsigned seed) = 0;

    // get bounds on reward function
    virtual std::pair< reward_t, reward_t > reward_range() const = 0;

    // for simulation (off-line learning)
    void restore_state(){
        simulation_state = current_state;
    }

    void save_state(const state_t &s){
        if (simulation) { simulation_state = s; }
        else { current_state = s; }
    }

    void set_simulation(bool state){
        simulation = state;
    }
    bool get_simulation() const{
        return simulation;
    }
};

