# pragma once

# include <algorithm>
# include <fstream>
# include <iostream>
# include <set>
# include <sstream>
# include "benchmarks/core.hpp"
# include "models/environment.hpp"
# include "utils/prng.hpp"


struct ResourceState {

    // agent position
    Coordinates position;

    // gold, gems 
    std::array< bool, 2 > flags;

    ResourceState( const Coordinates &pos,
                   const std::array< bool, 2 > &flags ) : position ( pos ), flags( flags ) {}

    bool operator< ( const ResourceState& other ) const {
        return ( ( position < other.position ) ||
                 ( ( position == other.position ) && ( flags < other.flags ) ) );
    }

    friend std::ostream &operator<<( std::ostream& os, const ResourceState &s ) {
        os << s.position << " Gold collected " << s.flags[0] << " gem collected: " 
           << s.flags[1];
        return os;
    }

    ResourceState() : position( 0, 0 ), flags( { false, false } ) {}

};


class ResourceGathering : public Environment< ResourceState, Direction, std::vector< double > > {

    double prob_of_attack = 0.0;

    using state_t = ResourceState;
    using action_t = Direction;
    using reward_t = std::vector< double >;

    size_t height, width;

    PRNG gen;
    std::set< Coordinates > gold;
    std::set< Coordinates > gems;
    std::set< Coordinates > attackers;

    ResourceState current_state, initial_state;


public:
    // environment interface
    ResourceState get_current_state() const override;
    std::map< ResourceState, double > get_transition( const ResourceState &pos, const Direction &dir ) const override;
    reward_t get_reward( const ResourceState &pos, const Direction &dir ) override;
    std::pair< reward_t, reward_t > reward_range() const override;
    std::vector< Direction > get_actions( const ResourceState &pos ) const override;
    Observation step( const Direction &dir ) override;
    Observation reset( unsigned seed ) override;
    std::string name() const override;
    void set_hyperparams( double prob );

    // constructors
    ResourceGathering();
    ResourceGathering( size_t height, size_t width, Coordinates initial_pos,
                                  const std::set< Coordinates > &gold,
                                  const std::set< Coordinates > &gems,
                                  const std::set< Coordinates > &attackers );

};
