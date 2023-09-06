# include "models/resource_gathering.hpp"

bool ResourceGathering::collides( const Coordinates& pos, Direction dir ) const {

    Coordinates next = pos + dir_to_vec( dir );

    if ( ( 0 > next.x ) || ( next.x >= width ) || 
         ( 0 > next.y ) || ( next.y >= height ) ) {
        return true;
    }

    return false;
}

Coordinates ResourceGathering::dir_to_vec( Direction dir ) const {
    int dx = 0, dy = 0;
    switch ( dir ) {
        case Direction::UP:
            dy = -1;
            break;
        case Direction::DOWN:
            dy = 1;
            break;
        case Direction::LEFT:
            dx = -1;
            break;
        case Direction::RIGHT:
            dx = 1;
            break;
    }
    return Coordinates( dx, dy );
}

ResourceState ResourceGathering::get_current_state() const {
    return current_state;
}


std::map< ResourceState, double > ResourceGathering::get_transition( const ResourceState &pos, const Direction &dir ) const {
    
    ResourceState successor( pos );
    successor.position += dir_to_vec( dir );

    if ( gold.find( successor.position ) != gold.end() ) {
        successor.flags[0] = true;
    }

    if ( gems.find( successor.position ) != gems.end() ) {
        successor.flags[1] = true;
    }

    // return to start with probability prob_of_attack
    if ( ( attackers.find( successor.position ) != attackers.end() ) &&  
           !approx_zero( prob_of_attack ) ) {
        return { { initial_state, prob_of_attack }, 
                 { successor, 1 - prob_of_attack } };
    }

    if ( successor.position == initial_state.position ) {
        // drop off gems & gold
        successor.flags = { false, false };
    }

    return { { successor, 1 } };
}


ResourceGathering::reward_t ResourceGathering::get_reward( const ResourceState &pos, const Direction &dir ) {
    auto transition = get_transition( pos, dir );


    reward_t rew = { 0, 0 };
    for ( const auto &[ state, prob ] : transition ) {
        if ( state.position == initial_state.position ) {
            // get rewards depending on collected
            rew[ 0 ] += pos.flags[ 0 ] * prob;
            rew[ 1 ] += pos.flags[ 1 ] * prob;

            //TODO: will need to adjust this for attacks
        }
    }

    return rew;
}

std::pair< ResourceGathering::reward_t, ResourceGathering::reward_t > ResourceGathering::reward_range() const {

    std::vector< double > min_vec, max_vec;
    min_vec = { 0.0, 0.0 };
    max_vec = { 1.0, 1.0 };

    return { min_vec, max_vec };
}

std::vector< Direction > ResourceGathering::get_actions( const ResourceState &pos ) const {

    std::vector< Direction > result;
    for ( auto dir : { Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT } ) {

        if ( !collides( pos.position, dir ) ){
            result.push_back( dir );
        }
    }

    return result;
}

ResourceGathering::Observation ResourceGathering::step( const Direction &dir )  {
    reward_t reward = get_reward( current_state, dir );

    // index of successor state in sparse matrix row 
    ResourceState next_state = gen.sample_distribution( get_transition( current_state, dir ) );
    
    current_state = next_state;
    return { next_state, reward, false };
}


ResourceGathering::Observation ResourceGathering::reset( unsigned seed )  {

    if ( seed == 0 ) {  gen.seed(); }
    else            {  gen.seed( seed ); }
    
    current_state = initial_state;

    return { initial_state , {0, 0}, false };
}

std::string ResourceGathering::name() const {
    return "Resource Gathering - " + std::to_string(prob_of_attack);
}

void ResourceGathering::set_hyperparams( double prob ) {
    prob_of_attack = prob;
}


ResourceGathering::ResourceGathering() : height( 5 ), width( 5 ),
    gold(), gems(), attackers(), current_state(), initial_state() {
}

ResourceGathering::ResourceGathering( size_t height, size_t width, 
                                      Coordinates initial_pos,
                                      const std::set< Coordinates > &gold,
                                      const std::set< Coordinates > &gems,
                                      const std::set< Coordinates > &attackers ) :
                                      height( height ), width ( width ),
                                      gold( gold ), gems( gems ), attackers( attackers ) { 
    initial_state = ResourceState( initial_pos, { false, false } );
    current_state = initial_state;
}
