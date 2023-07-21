#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <map>

/* 
 *  general environment interface to use when interacting with the solver
 *  (through environment handler object)
 */

template < typename state_t, typename action_t, typename reward_t >
class Environment{

public:
    // state after executing the step, reward value, 
    // and whether the state is terminal
    using Observation = std::tuple< state_t, reward_t, bool >;

    // get bounds on reward function
    virtual std::pair< reward_t, reward_t > reward_range() const = 0;


    virtual state_t get_current_state() const = 0;
    virtual std::map< state_t, double > get_transition(state_t state, 
                                                       action_t action) const = 0;
    virtual std::vector< action_t > get_actions(state_t state) const = 0;

    virtual reward_t get_reward(state_t s, action_t a) = 0;

    virtual Observation step(action_t) = 0;

    // enable potential reseeding of the envs prng, 0 signals default random init
    virtual Observation reset(unsigned seed=0) = 0;

    virtual std::string name() const = 0;

};

