# pragma once

# include <algorithm>
# include <fstream>
# include <iostream>
# include <set>
# include <sstream>
# include "models/environment.hpp"
# include "utils/prng.hpp"

/*
 * deep sea treasure benchmark for two objectives, see e.g.
 * https://github.com/imec-idlab/deep-sea-treasure
 * or the survey paper
 * https://ala2022.github.io/papers/ALA2022_paper_16.pdf
 */

struct Coordinates{

    int x, y;
    Coordinates() : x( 0 ), y( 0 ) {}
    Coordinates( int x, int y ) : x( x ), y( y ) {}
    Coordinates( std::pair< int, int > c ) : x( c.first ), y( c.second ) {}

    Coordinates& operator+=( const Coordinates& rhs ) {
        x += rhs.x;
        y += rhs.y;

        return *this;
    }

    Coordinates operator+( const Coordinates& rhs ) const {
        Coordinates copy = *this;
        copy += rhs;
        return copy;
    }

    Coordinates operator-=( const Coordinates& rhs ) {
        x -= rhs.x;
        y -= rhs.y;

        return *this;
    }

    Coordinates operator-( const Coordinates& rhs ) const {
        Coordinates copy = *this;
        copy -= rhs;
        return copy;
    }

    bool operator==( const Coordinates &rhs ) const {
        return x == rhs.x && y == rhs.y;
    }


    bool operator<( const Coordinates& rhs ) const {
        return ( x < rhs.x ) || ( ( x == rhs.x ) && ( y < rhs.y ) );
    }

    friend std::ostream &operator<<( std::ostream& os, const Coordinates &c ) {
        os << "(" << c.x << ", " << c.y << ")";
        return os;
    }

};

struct TreasureState {

    // agent position
    Coordinates position;


    // whether any treasure has been collected
    bool treasure_collected;


    bool operator< ( const TreasureState& other ) const {
        return ( ( position < other.position ) ||
                 ( ( position == other.position ) && ( treasure_collected < other.treasure_collected) ) );
    }

    friend std::ostream &operator<<( std::ostream& os, const TreasureState &s ) {
        os << s.position << " Collected: " << s.treasure_collected;
        return os;
    }

};

enum class Direction {
    UP, DOWN, LEFT, RIGHT
};

inline std::ostream &operator<<( std::ostream& os, const Direction &dir ) {
    switch ( dir ) {
        case Direction::UP:
            os << "UP";
            break;
        case Direction::DOWN:
            os << "DOWN";
            break;
        case Direction::LEFT:
            os << "LEFT";
            break;
        case Direction::RIGHT:
            os << "RIGHT";
            break;
    }
    return os;
}

class DeepSeaTreasure : public Environment< TreasureState, Direction, std::vector< double > > {

    using Obervation = Environment< Coordinates, Direction, std::vector< double > >::Observation;
    using reward_t = std::vector< double >;

    double noise = 0.0;
    double fuel_per_turn = -1;

    size_t height, width;
    PRNG gen;

    TreasureState current_state, initial_state;

    std::map< Coordinates, double > treasures;

    // some squares may be inacessible like in the original benchmark 
    std::set< Coordinates > inacessible_squares;

    // helper functions
    void initialize_state( Coordinates pos );
    Coordinates dir_to_vec( Direction dir ) const;
    bool collides( const Coordinates& pos, Direction dir ) const;
    bool terminated( const TreasureState& s );

public:
    // environment interface
    TreasureState get_current_state() const override;
    std::map< TreasureState, double > get_transition( const TreasureState &pos, const Direction &dir ) const override;
    reward_t get_reward( const TreasureState &pos, const Direction &dir ) override;
    std::pair< reward_t, reward_t > reward_range() const override;
    std::vector< Direction > get_actions( const TreasureState &pos ) const override;
    Observation step( const Direction &dir ) override;
    Observation reset( unsigned seed ) override;
    std::string name() const override;

    // getters and setters
    std::pair< size_t, size_t > get_dimensions() const;
    std::pair< double, double > get_hyperparams() const;
    std::map< Coordinates, double > get_treasures() const;
    std::set< Coordinates > get_inaccessible() const;
    std::pair< TreasureState, TreasureState > get_states() const;

    void set_hyperparams( double fuel, double noise_new );
    void from_file( const std::string &filename );

    // constructors
    DeepSeaTreasure();
    DeepSeaTreasure( size_t height, size_t width, Coordinates initial_state,
                     const std::map< Coordinates, double > &treasures,
                     const std::set< Coordinates > &inacessible_squares  );

    DeepSeaTreasure( const DeepSeaTreasure &other );
};
