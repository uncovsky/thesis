# pragma once

# include <algorithm>
# include <fstream>
# include <iostream>
# include <set>
# include <sstream>
# include "models/environment.hpp"
# include "benchmarks/core.hpp"
# include "utils/prng.hpp"

class FrozenLake : public Environment< Coordinates, Direction, std::vector< double > > {

    double prob_of_slipping = 0.33;

    using state_t = Coordinates;
    using action_t = Direction;
    using reward_t = std::vector< double >;

    size_t height, width;

    PRNG gen;
    std::set< Coordinates > pits;
    Coordinates current_state, initial_state;


public:

    // environment interface
    Coordinates get_current_state() const override;
    std::map< Coordinates, double > get_transition( const Coordinates &pos, const Direction &dir ) const override;
    std::vector< double > get_reward( const Coordinates &pos, const Direction &dir ) override;
    std::pair< reward_t, reward_t > reward_range() const override;
    std::vector< Direction > get_actions( const Coordinates &pos ) const override;
    Observation step( const Direction &dir ) override;
    Observation reset( unsigned seed ) override;
    std::string name() const override;
    void set_hyperparams( double prob );

    // constructors
    FrozenLake();
    FrozenLake( size_t height, size_t width, 
                const std::set< Coordinates > &pits,
                double prob_of_slipping );

};
