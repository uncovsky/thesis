# include "benchmarks/racetrack.hpp"

bool Racetrack::vehicle_collides( const VehicleState& state ) const{
    int x = state.position.x, y = state.position.y;

    if ( ( x < 0 ) || ( x >= width ) || ( y < 0 ) || ( y >= height ) ) {
        return true;
    }

    if ( collision_states.find( state.position ) != collision_states.end() ){
        return true;
    }

    return false;
}

VehicleState Racetrack::get_current_state() const {
    return current_state;
}


std::map< VehicleState, double > Racetrack::get_transition( const VehicleState &pos, 
                                                            const std::pair< int, int > &action ) const {

    VehicleState succ( pos ), succ_slip( pos );

    // transition with p=1 to terminal state if race is finished
    if ( goal_states.find( pos.position ) != goal_states.end() || pos == terminal_state ) {
        return { { terminal_state, 1.0 } };
    }

    succ.add_velocity( action );
    succ.move();
    succ_slip.move();

    std::map< VehicleState, double > result;

    // handle possible slips
    if ( vehicle_collides( succ ) ){
        result[ initial_state ] += 1 - slip_prob;
    }
    else {
        result[ succ ] += 1 - slip_prob;
    }

    if ( vehicle_collides( succ_slip ) ){
        result[ initial_state ] += slip_prob;
    }
    else{
        result[ succ_slip ] += slip_prob;
    }

    return result;
}

Racetrack::reward_t Racetrack::get_reward( const VehicleState &pos,
                                           const std::pair< int, int > &action ) {

    // if terminal state / moving to terminal state, don't lose fuel anymore
    if ( ( goal_states.find( pos.position ) != goal_states.end() ) || ( pos == terminal_state ) ){
        return { 0, 0 };
    }

    // return -1 * sum of velocities as fuel reward and -1 time rew
    double velocity_val = std::abs( pos.velocity.first + action.first ) + std::abs( pos.velocity.second + action.second );

    // always lose at least one fuel and time
    return { -1 * std::pow( velocity_val, 2 ) - 1, -1 };
}

std::pair< Racetrack::reward_t, Racetrack::reward_t > Racetrack::reward_range() const {
    std::vector< double > min_rew, max_rew;

    // can lose at most 11 fuel per turn
    min_rew = { -11, -1 };

    // after terminating race get max reward ( 0 in both comps )
    max_rew = { 0, 0 };

    return std::make_pair( min_rew, max_rew );
}

std::vector< Racetrack::action_t > Racetrack::get_actions( const VehicleState& pos ) const {
    
    // make all goal states transition to end with placeholder action
    if ( goal_states.find( pos.position ) != goal_states.end() ){
        return { { 0, 0 } };
    }

    // terminal state just self loops
    if ( pos == terminal_state ){
        return { { 0, 0 } };
    }

    std::vector< action_t > res;

    for ( int delta_x : { -1, 0, 1 } ) {
        for ( int delta_y : { -1, 0, 1 } ) {
            auto [ x, y ] = pos.velocity;

            // speed cannot exceed 5 in either component
            if ( ( std::abs( x + delta_x ) <= 5 ) && ( std::abs( y + delta_y ) <= 5 ) )
                res.emplace_back( delta_x, delta_y );
        }
    }

    return res;
}


Racetrack::Observation Racetrack::step( const Racetrack::action_t &dir )  {
    reward_t reward = get_reward( current_state, dir );

    // index of successor state in sparse matrix row 
    VehicleState next_state = gen.sample_distribution( get_transition( current_state, dir ) );
    
    current_state = next_state;
    return { next_state, reward, goal_states.find( next_state.position ) != goal_states.end() };
}


Racetrack::Observation Racetrack::reset( unsigned seed )  {

    if ( seed == 0 ) {  gen.seed(); }
    else            {  gen.seed( seed ); }
    
    current_state = initial_state;

    return { initial_state , {0, 0}, false };
}

std::string Racetrack::name() const {
    return "Racetrack" + std::to_string(height) + "x" + std::to_string(width) + std::to_string( slip_prob );
}

void Racetrack::set_hyperparams( double prob ) {
    slip_prob = prob;
}
void Racetrack::from_file( const std::string &filename ){

    std::ifstream input_str( filename );

    if ( input_str.fail() ) {
        throw std::runtime_error( "file " + filename + " does not exist");
    }
    
    std::string line;
    char token;
    size_t h = 0, w = 0;

    Coordinates initial_pos;
    bool start_defined = false;

    std::set< Coordinates > collisions;
    std::set< Coordinates > goals;

    while ( std::getline( input_str, line ) ){

        size_t idx = 0;
        std::stringstream ss( line );

        while ( ss >> token ) {

            if ( token == 'x' ) {
                collisions.emplace( idx, h );
            }

            else if ( token == 'g' ){
                goals.emplace( idx, h );
            }

            else if ( token == 's' ){
                if ( start_defined ){
                    throw std::runtime_error( "multiple starting states in" + filename );
                }
                initial_pos = Coordinates( idx, h );
                start_defined = true;
            }

            // otherwise free space
            else if ( token != '.' ){
                throw std::runtime_error( "invalid token in " + filename );
            }

            idx++;
        }

        if ( h == 0 ) { w = idx + 1; }
        else if ( idx + 1 != w ) {
            throw std::runtime_error( "invalid line width in " + filename );
        }
        h++;
    }

    if ( !start_defined ) {
        throw std::runtime_error( "no starting state in " + filename );
    }

    current_state = VehicleState();
    current_state.position = initial_pos;
    initial_state = current_state;

    height = h;
    width = w;
    collision_states = collisions;
    goal_states = goals;

}


Racetrack::Racetrack() : current_state()
                       , initial_state()
                       , terminal_state( Coordinates( -1, -1 ), { 0, 0 } )
                       , height( 1 )
                       , width( 4 )
                       , collision_states()
                       , goal_states( { Coordinates( 3, 0 ) } )
                       , gen() {}

