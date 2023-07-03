#include "seeder.hpp"
#include <limits>


std::uniform_int_distribution<unsigned> Seeder::seed_dist = std::uniform_int_distribution<unsigned>{0, std::numeric_limits<unsigned>::max()};

