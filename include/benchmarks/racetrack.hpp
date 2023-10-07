# pragma once

# include <algorithm>
# include <fstream>
# include <iostream>
# include <set>
# include <sstream>
# include "benchmarks/core.hpp"
# include "models/environment.hpp"
# include "utils/prng.hpp"


/* stochastic shortest path benchmark adapted to multiple dimensions
 * optimizing time & fuel simultaneously 
 * the state is given by a current position & velocity of the vehicle,
 * the possible actions include changing velocity by { -1, 0, 1 } in each
 * direction.
 * position of the vehicle is reset to initial state after a collision, and
 * velocity set to (0, 0)
 * each action taken has a certain probability of slipping, where the velocity
 * change does not take effect.
 * each move spends fuel equal to sum of velocities in each direction (
 * objective 2)
 * each move also takes one unit of time ( objective 1 )
 * the goal of the agent is to reach one of the terminal states ( the finish
 * line )
 */

// state is vehicle position and velocity in x & y direction
struct VehicleState {

    Coordinates position = Coordinates( 0, 0 );
    std::pair< int , int > velocity = { 0, 0 };

    void add_velocity( const std::pair< int, int > &velocity_change ) {
        velocity.first += velocity_change.first;
        velocity.second += velocity_change.second;
    }

    void move() {
        position.x += velocity.first;
        position.y += velocity.second;
    }

    bool operator==( const VehicleState &other ) const {
        return ( position == other.position ) && ( velocity == other.velocity );
    }

    bool operator<( const VehicleState &other ) const{
        return ( position < other.position ) || 
               ( ( position == other.position ) && ( velocity < other.velocity ) ) ;
    }


    friend std::ostream &operator<<( std::ostream& os, const VehicleState &s ) {
        os << s.position << " Speed: " << s.velocity.first << "; " << s.velocity.second;
        return os;
    }
    
    VehicleState( const Coordinates &pos, const std::pair< int, int > &velo ) : position( pos )
                                                                              , velocity( velo ) {  }

    VehicleState() : position( )
                   , velocity( ) {  }
};

// for trace output during trajectory sampling
inline std::ostream &operator<<( std::ostream& os, const std::pair< int, int > &velocity ) {
    os << velocity.first << "; " << velocity.second;
    return os;
}

class Racetrack : public Environment< VehicleState, std::pair< int, int >,  std::vector< double > > {

    VehicleState current_state, initial_state;

    // dedicated state ( race finished )
    VehicleState terminal_state;

    size_t height, width;

    std::set< Coordinates > collision_states;
    std::set< Coordinates > goal_states;
    double slip_prob = 0.1;
    PRNG gen;

    bool vehicle_collides( const VehicleState& state ) const;

public:
    using state_t  = Coordinates;
    using action_t = std::pair< int, int >;
    using reward_t = std::vector< double >;
    // environment interface
    VehicleState get_current_state() const override;
    std::map< VehicleState, double > get_transition( const VehicleState &pos, const action_t &dir ) const override;
    reward_t get_reward( const VehicleState &pos, const action_t &dir ) override;
    std::pair< reward_t, reward_t > reward_range() const override;
    std::vector< action_t > get_actions( const VehicleState &pos ) const override;
    Observation step( const action_t &act ) override;
    Observation reset( unsigned seed ) override;
    std::string name() const override;

    // getters and setters
    std::pair< size_t, size_t > get_dimensions() const;
    std::pair< double, double > get_hyperparams() const;

    void set_hyperparams( double prob );
    void from_file( const std::string &filename );

    // constructors
    Racetrack();
};

