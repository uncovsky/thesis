#include "models/mdp.hpp"
#include "solvers/brtdp.hpp"
#include "geometry/polygon.hpp"
#include "models/env_wrapper.hpp"
#include <iostream>
#include "geometry/pareto.hpp"


int main(){

    Matrix3D< int > rewards;
    Matrix3D< double > transitions;


    // add two matrices (state transitions)
    transitions.push_back( Matrix2D< double >( 2, 2 ) );
    transitions.push_back( Matrix2D< double >( 2, 2 ) );
    rewards.push_back( Matrix2D< int >( 2, 2 ) );


    // from 0 to 0,1 by action 0 with 0.5 probability
    transitions[0].coeffRef( 0, 0 ) = 0.5;
    transitions[0].coeffRef( 0, 1 ) = 0.5;
    transitions[0].coeffRef( 1, 1 ) = 1.0;
    transitions[1].coeffRef( 1, 1 ) = 1.0;

    rewards[0].coeffRef( 0, 0 ) = 1;
    rewards[0].coeffRef( 0, 1 ) = 2;
    rewards[0].coeffRef( 1, 0 ) = 0;

    // third argument is the reward bounds, just placeholders right now
    MDP<int> mdp( transitions,
                  rewards,
                  std::make_pair ( std::vector< int > ( { 0 } ), std::vector< int > ( { 7 } ) ),
                  0
                );


    EnvironmentWrapper< size_t, size_t, std::vector<int>, int> env_wrap( &mdp );


    // sanity checks, todo some actual tests
    auto [ s, r, termination ] = mdp.reset(2380);

    size_t i = 1;
    while (!termination){
        auto res = mdp.step(0);
        s = std::get<0>(res);
        r = std::get<1>(res);
        termination = std::get<2>(res);

        std::cout << "step " << i++ << " curr state: " << s << " terminated: " << termination << "\n";
    }


    std::vector< size_t > actions = mdp.get_actions(0);

    std::cout << "Available actions from state " << s << ": ";
    for ( size_t act : actions ) {
        std::cout << act << " ";
    }

    std::map < size_t, double > succ = mdp.get_transition( 0, 1 );

    std::cout << "Available transitions from state 0 under action 1: \n";
    for ( auto [ s, p ] : succ ) {
        std::cout << "state " << s << " with prob " << p << std::endl;
    }

    std::cout << "state: " << s << " terminated: " << termination << " reward: ";
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

    Polygon<double> pc( { { 0, 1 }, {1, 0}, {1, 1}, {0, 0}, {0.5, 0.5}, {0.25, 0.75}, {0.3, 0.7} } );
    // copy.scalar_multiply( 0.25 );
    // pc.add_curve(copy);
    pc.write_to_file("pareto_before.txt");
    pc.convex_hull();
    pc.write_to_file("pareto_test.txt");
        
}
