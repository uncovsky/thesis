# include "benchmarks/frozen_lake.hpp"

Coordinates FrozenLake::get_current_state() const {
    return current_state;
}

std::map< Coordinates, double > FrozenLake::get_transition( const Coordinates &pos, const Direction &dir ) const {

    if ( pos == Coordinates( height -1 , width - 1 ) ) {
        return { std::make_pair( pos, 1.0 ) };
    }
    
    if ( prob_of_slipping == 0 ) {
        return { std::make_pair( pos + dir_to_vec( dir ) , 1 - prob_of_slipping ) };
    } 

    std::vector< Direction > slip_directions = valid_perpendicular( pos, dir, height, width );

    // divide by # of possible slips
    double distr_prob = prob_of_slipping / slip_directions.size();

    std::map< Coordinates, double > res = { std::make_pair( pos + dir_to_vec( dir ) , 1 - prob_of_slipping ) };

    for ( Direction slip : slip_directions ){
        res[ pos + dir_to_vec( slip ) ] = distr_prob;
    }

    return res;
};


std::vector< double > FrozenLake::get_reward( const Coordinates &pos, const Direction &dir ){

    // prevent collecting treasure multiple times
    if ( pos == Coordinates( height - 1, width - 1 ) ) 
        return { 0, 0 };

    auto transition = get_transition( pos, dir );

    std::vector< double > rewards( 2, 0.0 );

    for ( const auto &[ succ, prob ] : transition ){
        if ( succ == Coordinates( height - 1, width - 1 ) ) {
            // add 1 * prob reward if hit target
            rewards[0] += prob;
        }

        else if ( pits.find( succ ) != pits.end() ) {
            // negative rew for pits
            rewards[1] -= prob;
        }
    }

    return rewards;
}

std::pair< std::vector< double >, std::vector< double > > FrozenLake::reward_range() const {
    std::vector< double > min_vec = { 0, -1 }, max_vec = { 1, 0 };
    return std::make_pair( min_vec, max_vec );
}

std::vector< Direction > FrozenLake::get_actions( const Coordinates& pos ) const {
    std::vector< Direction > result;
    for ( auto dir : { Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT } ) {

        if ( !collides( pos, dir, height, width ) ){
            result.push_back( dir );
        }
    }

    return result;
}


FrozenLake::Observation FrozenLake::step( const Direction &dir )  {
    reward_t reward = get_reward( current_state, dir );

    // index of successor state in sparse matrix row 
    Coordinates next_state = gen.sample_distribution( get_transition( current_state, dir ) );
    
    current_state = next_state;
    return { next_state, reward, false };
}


FrozenLake::Observation FrozenLake::reset( unsigned seed )  {

    if ( seed == 0 ) {  gen.seed(); }
    else            {  gen.seed( seed ); }
    
    current_state = initial_state;

    return { initial_state , {0, 0}, false };
}


std::string FrozenLake::name() const {
    std::string dimensions = std::to_string( height ) + " x " + std::to_string( width );
    return "FrozenLake " + dimensions + " " + std::to_string( prob_of_slipping );
}

void FrozenLake::set_hyperparams( double prob ) {
    prob_of_slipping = prob;
}

FrozenLake::FrozenLake() : height( 9 )
                         , width( 9 )
                         , pits( { Coordinates( 0, 6 )
                                 , Coordinates( 1, 7 )
                                 , Coordinates( 3, 1 )
                                 , Coordinates( 3, 3 )
                                 , Coordinates( 7, 5 ) } )
                         , current_state( 0, 0 )
                         , initial_state( 0, 0 )
{
    /* initalizes to sample benchmark from tutorial
     * https://gymnasium.farama.org/tutorials/training_agents/FrozenLake_tuto
     */
}
FrozenLake::FrozenLake( size_t height
                      , size_t width
                      , const std::set< Coordinates > &pits
                      , double prob_of_slipping ) :
                                                    prob_of_slipping ( prob_of_slipping )
                                                  , height( height )
                                                  , width( width )
                                                  , pits( pits )
                                                  , current_state( 0, 0 )
                                                  , initial_state( 0, 0 )
{

}
