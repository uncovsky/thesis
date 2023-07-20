#include "models/mdp.hpp"
#include "models/env_wrapper.hpp"
#include <iostream>
#include "geometry/pareto.hpp"


int main(){

    Matrix3D< int > rewards;
    Matrix3D< double > transitions;


    // add 1 by 1 matrices
    transitions.push_back( Matrix2D< double >( 2, 2 ) );
    rewards.push_back( Matrix2D< int >( 2, 2 ) );


    // from 0 to 0,1 by action 0 with 0.5 probability
    transitions[0].coeffRef( 0, 0 ) = 0.5;
    transitions[0].coeffRef( 0, 1 ) = 0.5;
    transitions[0].coeffRef( 1, 1 ) = 1.0;

    rewards[0].coeffRef( 0, 0 ) = 1;
    rewards[0].coeffRef( 0, 1 ) = 0;
    rewards[0].coeffRef( 1, 1 ) = 2;

    // third argument is the reward bounds, just placeholders right now
    MDP<int> mdp( transitions,
                  rewards,
                  std::make_pair ( std::vector< int > ( { 0 } ), std::vector< int > ( { 7 } ) ),
                  0
                );


    EnvironmentWrapper< size_t, size_t, std::vector<int>, int> env_wrap( &mdp );


    // sanity checks, todo some actual tests, and fix mdp step function so it works
    // properly
    auto [ s, r, end ] = mdp.reset(0);

    std::vector< size_t > actions = mdp.get_actions(0);

    std::cout << "Available actions from state " << s << ": ";
    for ( size_t act : actions ) {
        std::cout << act << " ";
    }

    std::cout << "state: " << s << " terminated: " << end << " reward: ";
    for ( const auto& x : r ){
        std::cout << x << ", ";
    }

    std::cout << strictly_non_dominated<double>({2, 3, 6}, {2, 3, 4}) << std::endl;
    std::cout << strictly_non_dominated<double>({1, 3, 6}, {2, 3, 4}) << std::endl;
    std::cout << strictly_non_dominated<double>({2, 3, 4}, {2, 3, 4}) << std::endl;
    std::cout << strictly_non_dominated<double>({2, 3, 2}, {2, 3, 4}) << std::endl;

    std::set< std::vector< double > > s1 = { {1, 2, 3}, {2, 3, 4} };
    std::set< std::vector< double > > s2 = { {1, 2, 2}, {3, 1, 2} };

    nondominated_union(s1, s2);

    for (const auto &x : s1){
        for (const auto &y : x)
            std::cout << y;
        std::cout << "\n";
    }
        
}
