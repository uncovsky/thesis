# pragma once

#include <algorithm>
#include <set>
#include "models/environment.hpp"
#include "utils/prng.hpp"

/*
 * deep sea treasure benchmark in c++, see e.g.
 * https://github.com/imec-idlab/deep-sea-treasure
 * or the survey paper
 * https://ala2022.github.io/papers/ALA2022_paper_16.pdf
 */

struct Coordinates{

    int x, y;

    Coordinates( int x, int y ) : x( x ), y( y ) {}
    Coordinates( std::pair< int, int > c ) : x( c.first ), y( c.second ) {}

};

enum class Direction {
    UP, DOWN, LEFT, RIGHT
};

class DeepSeaTreasure : Environment< Coordinates, Direction, std::pair< double, double > > {

    using Obervation = Environment< Coordinates, Direction, std::pair< double, double > >::Observation;
    using reward_t = std::pair< double, double >;

    double noise = 0.0;
    double fuel_per_turn = -1;

    size_t height, width;
    PRNG gen;

    Coordinates current_state, initial_state;

    std::map< Coordinates, double > treasures;

    // some squares may be inacessible like in the original benchmark 
    std::set< Coordinates > inacessible_squares;

    Coordinates move( const Coordinates &pos, Direction dir ) const {
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
        return Coordinates( pos.x + dx, pos.y + dy );
    }

    bool collides( const Coordinates& pos, Direction dir ) const {
        Coordinates next = move( pos, dir );

        if ( inacessible_squares.find( next ) != inacessible_squares.end() )
            return true;
        if ( ( 0 > next.x ) || ( next.x >= width ) || 
                ( 0 > next.y ) || ( next.y >= height ) ) {
            return true;
        }

        return false;
    }

    bool terminated(){
        return treasures.empty();
    }

public:
    Coordinates get_current_state() const override {
        return current_state;
    }

    std::map< Coordinates, double > get_transition( Coordinates pos, Direction dir ) const override {
        std::vector< Direction > actions = get_actions( pos );
        std::vector< Coordinates > successors;
        for ( const auto &a : actions ) {
            successors.push_back( move( pos, a ) );
        }

        /* distribute noise to unselected successors
         * assuming dir is a possible action in pos
         */ 
        double noise_div = noise / ( successors.size() - 1 );

        std::map< Coordinates, double > transition;

        for ( size_t i = 0; i < actions.size(); i++ ) {
            if ( actions[i] == dir ) { transition[ successors[i] ] = 1.0 - noise;  }
            else { transition[ successors[i] ] = noise_div; }
        }

        return transition;
    }


    reward_t get_reward( Coordinates pos, Direction dir ) override {

        double treasure = 0;
        auto transition = get_transition( pos, dir );

        // weigh state rewards of sucessors 
        for ( const auto &[ successor, prob ] : transition ) {
            if ( treasures.find( successor ) != treasures.end() ) {
                treasure += prob * treasures[ successor ];
            }
        }

        return std::make_pair( treasure, fuel_per_turn );
    }

    reward_t reward_range(){
        auto max_it = std::max_element( treasures.begin(), treasures.end(),
                                        []( auto x, auto y ) { return x.second < y.second; });

        return std::make_pair( max_it->second, fuel_per_turn );
    }


    std::vector< Direction > get_actions( Coordinates pos ) const override {
        std::vector< Direction > result;
        for ( auto dir : { Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT } ) {

            if ( !collides( pos, dir ) ){
                result.push_back( dir );
            }
        }

        return result;
    }


    Observation step( Direction dir ) override {
        reward_t reward = get_reward( current_state, dir );

        // index of successor state in sparse matrix row 
        Coordinates next_state = gen.sample_distribution( get_transition( current_state, dir ) );
        
        current_state = next_state;
        return { next_state, reward, terminated() };
    }

    Observation reset( unsigned seed ) override{

        if ( seed == 0 ) {  gen.seed(); }
        else            {  gen.seed( seed ); }
        
        current_state = initial_state;

        return { initial_state , {0, 0}, terminated() };
    }
};
