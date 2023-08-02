#include "models/mdp.hpp"
#include "solvers/brtdp.hpp"
#include "geometry/polygon.hpp"
#include "models/env_wrapper.hpp"
#include <iostream>
#include "geometry/pareto.hpp"


int main(){

    Matrix2D< double > hello;
    // fix this , any way to not allocate all beforehand?
    // check out reserve function(nnz) TODO
    hello.resize( 1000 , 1000 );

    std::vector< Eigen::Triplet< double > > triplets;
    Eigen::Triplet hi ( 1, 1, 1.0 );

    triplets.push_back( hi );
    hello.setFromTriplets( triplets.begin(), triplets.end() );

    std::cout << hello;

}
