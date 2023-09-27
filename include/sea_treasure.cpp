# include <algorithm>
# include "benchmarks/sea_treasure.hpp"

void DeepSeaTreasure::initialize_state( Coordinates pos ) {
    TreasureState res;

    res.position = pos;
    res.treasure_collected = false;

    initial_state = res;
    current_state = res;
}


bool DeepSeaTreasure::terminated( const TreasureState& s ){
    return s.treasure_collected;
}


TreasureState DeepSeaTreasure::get_current_state() const  {
    return current_state;
}


std::map< TreasureState, double > DeepSeaTreasure::get_transition( const TreasureState &s, const Direction &dir ) const  {

    // terminal state
    if ( s.treasure_collected ) {
        return { std::make_pair( s, 1.0 ) };
    }

    std::vector< Direction > actions = get_actions( s );
    std::vector< TreasureState > successors;
    for ( const auto &a : actions ) {

        TreasureState copy(s);
        copy.position += dir_to_vec( a );
        if ( treasures.find( copy.position ) != treasures.end() ) {
            copy.treasure_collected = true;
        }

        successors.emplace_back( std::move( copy ) );
    }

    /* distribute noise to unselected successors
     * assuming dir is a possible action in pos
     */ 
    double noise_div = noise / ( successors.size() - 1 );

    std::map< TreasureState, double > transition;

    for ( size_t i = 0; i < actions.size(); i++ ) {
        if ( actions[i] == dir ) { transition[ successors[i] ] = 1.0 - noise;  }
        else if ( !approx_zero( noise_div ) ) { transition[ successors[i] ] = noise_div; }
    }

    return transition;
}


DeepSeaTreasure::reward_t DeepSeaTreasure::get_reward( const TreasureState &s, const Direction &dir )  {

    double treasure = 0;
    auto transition = get_transition( s, dir );

    if ( terminated( s ) ) {
        return { 0, 0 };
    }

    // weigh state rewards of sucessors 
    for ( const auto &[ successor, prob ] : transition ) {
        auto it = treasures.begin();
        for ( size_t i = 0; i < treasures.size(); i++ ) {
            // if visiting treasure for the first time
            if ( ( it->first == successor.position ) && ( !s.treasure_collected ) ) {
                treasure += it->second * prob;
            }
            it++;
        }
    }

    return { treasure, fuel_per_turn };
}


std::pair< DeepSeaTreasure::reward_t, DeepSeaTreasure::reward_t > DeepSeaTreasure::reward_range() const  {
    auto cmp = [] ( auto x, auto y ) { return x.second < y.second; };
    auto [ min_it, max_it ] = std::minmax_element( treasures.begin(), treasures.end(), cmp );

    // check if all non-starting squares are treasures potentially
    double min_treasure = ( width * height - 1 ) == treasures.size() ? min_it->second : 0;

    std::vector< double > min_vec = { min_treasure, fuel_per_turn };
    std::vector< double > max_vec = { max_it->second, 0 };

    // TODO: change later
    min_vec = { 0, -1 };
    max_vec = { 1, 0 };

    return std::make_pair( min_vec, max_vec );
}


std::vector< Direction > DeepSeaTreasure::get_actions( const TreasureState &s ) const  {
    std::vector< Direction > result;
    for ( auto dir : { Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT } ) {

        if ( !collides( s.position, dir, height, width ) ){
            result.push_back( dir );
        }
    }

    return result;
}


DeepSeaTreasure::Observation DeepSeaTreasure::step( const Direction &dir )  {
    reward_t reward = get_reward( current_state, dir );

    // index of successor state in sparse matrix row 
    TreasureState next_state = gen.sample_distribution( get_transition( current_state, dir ) );
    
    current_state = next_state;
    return { next_state, reward, terminated( next_state ) };
}


DeepSeaTreasure::Observation DeepSeaTreasure::reset( unsigned seed )  {

    if ( seed == 0 ) {  gen.seed(); }
    else            {  gen.seed( seed ); }
    
    current_state = initial_state;

    return { initial_state , {0, 0}, false };
}


std::string DeepSeaTreasure::name() const  {
    return "Deep Sea Treasure";
}


std::pair< size_t, size_t > DeepSeaTreasure::get_dimensions() const {
    return { height, width };
}


std::pair< double, double > DeepSeaTreasure::get_hyperparams() const {
    return { noise, fuel_per_turn };
}


std::map< Coordinates, double > DeepSeaTreasure::get_treasures() const {
    return treasures;
}


std::pair< TreasureState, TreasureState> DeepSeaTreasure::get_states() const {
    return std::make_pair( current_state, initial_state );
}

std::set< Coordinates > DeepSeaTreasure::get_inaccessible() const {
    return inacessible_squares;
}


void DeepSeaTreasure::set_hyperparams( double fuel, double noise_new ) {
    fuel_per_turn = fuel;
    noise = noise_new;
}


void DeepSeaTreasure::from_file( const std::string &filename ) {

    std::ifstream input_str( filename );

    if ( input_str.fail() ) {
        throw std::runtime_error( "file " + filename + " does not exist");
    }
    
    std::string line, token;
    size_t h = 0, w = 0;

    std::set< Coordinates > blocked;
    std::map< Coordinates, double > treasures_new;

    while ( std::getline( input_str, line ) ){

        size_t idx = 0;
        std::stringstream ss( line );

        while ( ss >> token ) {
            if ( token == "*" ) {
                blocked.emplace( idx, h );
                idx++;
                continue;
            }

            if ( token == "#" ) {
                idx++;
                continue;
            }


            double val = 0;

            try {
                val = std::stod( token );
            }

            catch ( ... ) {
                throw std::runtime_error( "invalid token in " + filename );
            }

            treasures_new[ Coordinates( idx, h ) ] = val;
            idx++;
        }

        if ( h == 0 ) { w = idx + 1; }
        else if ( idx + 1 != w ) {
            throw std::runtime_error( "invalid line width in " + filename );
        }
        h++;
    }

    height = h;
    width = w;
    inacessible_squares = blocked;
    treasures = treasures_new;

    // just use 0,0 as starting implicitly for now 
    initialize_state( Coordinates(0, 0) );
}


DeepSeaTreasure::DeepSeaTreasure( size_t height, size_t width, Coordinates initial_pos,
                                  const std::map< Coordinates, double > &treasures,
                                  const std::set< Coordinates > &inacessible_squares  ) : 
                        height( height ), width( width ),
                        current_state(),
                        initial_state(),
                        treasures( treasures ),
                        inacessible_squares( inacessible_squares ) { initialize_state( initial_pos ); }

// default init position to ( 0, 0 )
DeepSeaTreasure::DeepSeaTreasure() : height(0),
                                     width(0),
                                     current_state(),
                                     initial_state(),
                                     treasures(),
                                     inacessible_squares() { initialize_state( Coordinates(0, 0) ); }

DeepSeaTreasure::DeepSeaTreasure( const DeepSeaTreasure &other ) {
    auto [ current, initial ] = other.get_states();
    auto [ h, w ] = other.get_dimensions();
    auto [ n, fuel ] = other.get_hyperparams();
    std::map< Coordinates, double > other_treasures = other.get_treasures();
    std::set< Coordinates > inaccess = other.get_inaccessible();

    height = h;
    width = w;
    noise = n;
    fuel_per_turn = fuel;
    treasures = other_treasures;
    inacessible_squares = inaccess;
    current_state = current;
    initial_state = initial;
}

