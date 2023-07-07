#include "models/mdp.hpp"
#include "models/experience.hpp"
#include <iostream>


int main(){

    Eigen::SparseMatrix<double, Eigen::RowMajor> sex;
    MDP<int> ahoj;

    EnvironmentWrapper< size_t, size_t, int, int> env_wrap(&ahoj);

    auto [s, a, r, succ] = ahoj.reset(10);

    std::cout << s << "\n" << a << "\n" << r << "\n" << succ;
    std::cout << ahoj.name();

}
