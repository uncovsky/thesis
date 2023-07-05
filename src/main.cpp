#include "models/mdp.hpp"
#include <iostream>


int main(){

    Eigen::SparseMatrix<double, Eigen::RowMajor> sex;
    MDP<size_t> ahoj;

    auto [s, a, r, succ] = ahoj.reset(10);

    std::cout << s << "\n" << a << "\n" << r << "\n" << succ;
    std::cout << ahoj.name();

}
