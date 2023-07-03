#pragma once

#include <vector>
#include "models/environment.hpp"
#include "seeder.hpp"
#include "Eigen/Dense"

using Matrix2D = Eigen::Matrix< double, 
                                Eigen::Dynamic, 
                                Eigen::Dynamic, 
                                Eigen::RowMajor | Eigen::AutoAlign >;

using Matrix3D = std::vector< Matrix2D >;

template < typename reward_t >
class MDP : public Environment< size_t, size_t , reward_t > {

    using typename Environment< size_t, size_t, reward_t > :: Observation;


    // single gamma so far
    double gamma;
    
    size_t starting_state;

    // reward bounds W(s,a)_i .. 
    std::pair< reward_t, reward_t > reward_bounds;

    // S x A x S \delta(s,a,s')
    Matrix3D transitions;

    // S x A
    Matrix2D rewards;

    Matrix2D availActions;

public:

    MDP(const Matrix3D& transitions, const Matrix2D& rewards, size_t s, double gamma = 1)
            : transitions(transitions),
              rewards(rewards),
              starting_state(s),
              gamma(g) {}

    MDP(Matrix3D&& transitions, Matrix2D&& rewards, size_t s, double gamma = 1)
            : transitions(std::move(transitions)),
              rewards(std::move(rewards)),
              starting_state(s),
              gamma(g) {}


    std::vector< size_t > get_actions(size_t state){
        return transitions[state].row()
    }

    virtual Observation step(size_t a) override {

    }

    virtual 


};



