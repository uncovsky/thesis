#pragma once

#include <vector>
#include "models/environment.hpp"
#include "utils/prng.hpp"
#include "Eigen/Core"
#include "Eigen/SparseCore"

using Matrix2D = Eigen::SparseMatrix< double, 
                                Eigen::RowMajor >;

using Matrix3D = std::vector< Matrix2D >;

template < typename reward_t >
class MDP : public Environment< size_t, size_t , reward_t > {

    using typename Environment< size_t, size_t, reward_t > :: Observation;
    
    size_t initial_state, current_state;

    // reward bounds W(s,a)_i .. 
    std::pair< reward_t, reward_t > reward_bounds;

    // S x A x S \delta(s,a,s')
    // since row major actions will be in rows
    Matrix3D transitions;

    // S x A
    Matrix2D rewards;

    PRNG gen;

public:

    MDP() : transitions(),
            rewards(),
            reward_bounds({reward_t(), reward_t()}),
            initial_state(0),
            current_state(0){}

    MDP(const Matrix3D& transitions, const Matrix2D& rewards, 
              std::pair< reward_t, reward_t > reward_bounds, size_t s)
            : transitions(transitions),
              rewards(rewards),
              reward_bounds(reward_bounds),
              initial_state(s),
              current_state(s){}

    // move constructor
    MDP(Matrix3D&& transitions, Matrix2D&& rewards, 
              std::pair< reward_t, reward_t > reward_bounds, size_t s)
            : transitions(std::move(transitions)),
              rewards(std::move(rewards)),
              reward_bounds(reward_bounds),
              initial_state(s),
              current_state(s){}


    std::vector< size_t > get_actions(size_t state) const override{

        std::vector< size_t > available_actions;


        /* Get pointer to an array including the number of nonzero elements in
         * each row. An action i is available in given state, iff the sparse
         * matrix includes a nonzero number of entries at given row i. */
        auto it = transitions[state].innerNonZeroPtr();

        for (size_t i = 0; i < transitions[state].outerSize(); ++i){
            if (*it > 0) { available_actions.push_back(i); }
            it++;
        }

        return available_actions;
    }

    std::map< size_t, double > get_transition(size_t state, 
                                              size_t action) const override {

        std::map< size_t, double > delta;

        // iterate over every entry in the row corresponding to delta(s,a)
        for (Matrix2D::InnerIterator it(transitions[state], action); it; ++it){
            delta.emplace( it.col(), it.value() );
        }

        return delta;
    }


    std::pair< reward_t, reward_t > reward_range() const override {
        return reward_bounds;
    }

    reward_t get_expected_reward(size_t state, size_t action){
        return rewards.coeffRef(state, action);
    }


    std::string name() const override {
        return "Sparse-MDP";
    }

    Observation step(size_t action) override {

        // get iterator to probabilities of successor states
        Matrix2D::InnerIterator it(transitions[current_state], action);

        reward_t reward = get_expected_reward(current_state, action);

        // index of successor state in sparse matrix row 
        size_t next_state = gen.sample_distribution(get_transition(current_state, action));

        current_state = next_state;

        return {current_state, action, reward, next_state};
    }


    Observation reset(unsigned seed) override{
        if (seed = 0)
            gen.seed();
        else
            gen.seed(seed);
        
        current_state = initial_state;

        return {current_state, 0, reward_t(), current_state};
    }
};



