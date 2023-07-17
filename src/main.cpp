#include "models/mdp.hpp"
#include "models/experience.hpp"
#include <iostream>
#include "geometry/polytope.hpp"


int main(){

    Eigen::SparseMatrix<double, Eigen::RowMajor> mat;
    MDP<double> ahoj;

    EnvironmentWrapper< size_t, size_t, double, int> env_wrap(&ahoj);

    auto [s, a, r, succ] = ahoj.reset(10);

    //std::cout << s << "\n" << a << "\n" << r << "\n" << succ;
    std::cout << ahoj.name();

    std::cout << strictly_non_dominated<double>({2, 3, 6}, {2, 3, 4}) << std::endl;
    std::cout << strictly_non_dominated<double>({2, 3, 4}, {2, 3, 4}) << std::endl;
    std::cout << strictly_non_dominated<double>({2, 3, 2}, {2, 3, 4}) << std::endl;


    std::set< std::vector< double > > s1 = { {1, 2, 3}, {2, 3, 4} };
    std::set< std::vector< double > > s2 = { {1, 2, 2}, {3, 1, 2} };

    nondominated_union(s1, s1);

    for (const auto &x : s1){
        for (const auto &y : x)
            std::cout << y;
        std::cout << "\n";
    }
        
}
