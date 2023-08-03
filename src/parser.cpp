# include "parser.hpp"

/* first pass
 * find the number of states and for each state the number of associated
 * actions and successors
 *
 * validate the state names ( non-negative integers )
 *  - map them to indices
 * validate the probabilities ( floats between 0.0 - 1.0 )
 * validate that SxAxS triplets don't repeat
 * accumulate the necessary triplets ? 
 *
 * second pass
 *  build the model from triplets
 */


bool PrismParser::eol() const {
    return curr == end;
}

char PrismParser::get_token() {
    return eol() ? '\0' : *curr;
}



// accepts strings of digits of length >= 1 ( initial zeroes allowed )
std::string PrismParser::load_number( ) {
    std::string id;
    char token = get_token();
    require( std::isdigit() );
    id.push_back( token );

    token = get_token();

    while ( check( std::isdigit() ) ) {
        id.push_back( token );
        token = get_token();
    }

    return id;
}

// accepts and loads a floating point number
double PrismParser::load_float(){
    std::string flt;
    std::string decimal;

    flt.push_back( get_token() );
    require( std::isdigit() );
    
    if ( check( '.' ) ){
        decimal = load_number();
        flt += '.' + decimal;
    }

    return std::stod( flt );
}

// names of states & actions in the file are mapped to first available indices
// which are then used to construct the triplets, flag signalizes whether a
// state or an action is to be translated
size_t PrismParser::translate( const std::string& name, bool state ){

    size_t id = string_to_ull( name );
    std::map< size_t, size_t > &target = state ? state_to_index : action_to_index;

    // first available index;
    size_t index = target.size();

    if ( target.find( id ) == target.end() ) {
        target[ id ] = index;
    }

    // translate
    return target[ id ];
}

std::tuple< size_t, size_t, size_t > PrismParser::match_triplet(){
    remove_all( std::isspace() );

    std::string s = load_number();
    remove_all( std::isspace() );

    std::string a = load_number();
    remove_all( std::isspace() );

    std::string succ = load_number();
    remove_all( std::isspace() );

    // convert relevant info to numbers
    size_t s_id = translate( s, true );
    size_t succ_id = translate( succ, true );
    size_t a_id = translate( a, false );

    return { s_id, a_id, succ_id };
}


void PrismParser::match_transition( ){

    auto [ s_id, a_id, succ_id ] = match_triplet();
    double p = load_float();


    if ( ( prob <= 0 ) || ( prob > 1 ) ) {
        throw ParsingError( line_num, "Invalid transition probability\n" );
    }

    remove_all( std::isspace() );
    require( '\0' );

    transition_info[ s_id ].add_triplet( a_id, succ_id, p );
}

void PrismParser::match_reward( ){
    auto [ s_id, a_id, succ_id ] = match_triplet();

    if ( ( transition_info.find( s_id ) == transition_info.end() ) ||
         ( !transition_info.contains( a_id, succ_id ) ) ) {
        throw ParsingError( line_num, "This reward transition is not present in the transition file.\n" )
    }

    double reward = load_float();
    remove_all( std::isspace() );
    require( '\0' );
    
    reward_structures.back()[ s_id ].add_triplet( a_id, succ_id, reward );
}


void PrismParser::parse_transition_file( const std::string &filename ){
    std::ifstream input_str( filename );

    while ( std::getline( input_str ), line ) {
        line_num++;
        // set iterators
        curr = line.begin();
        end  = line.end();
        // match transition
        match_transition();
    }

    // TODO: could maintain reverse mappings ( from indices to original state
    // names in the file ) for better error messages
    for ( const auto &[ id, data ] : transition_info ){
        if ( !data.valid_probabilities() ){
            throw ParsingError(0, "invalid transition probabilities for state mapped to index " + id + " \n");
        }
    }
}


void PrismParser::parse_reward_file( const std::string &filename ){

    reward_structures.push_back({});
    std::ifstream input_str( filename );

    while ( std::getline( input_str ), line ) {
        line_num++;
        curr = line.begin();
        end  = line.end();
        match_reward();
    }

    // TODO: set misssing rewards as zero
}


bool PrismParser::check( int (* callback)( char ) ){
    if callback( get_token() ) {
        curr++;
        return true;
    }

    return false;
}


bool PrismParser::check( char token ){
    if ( token == get_token() ) {
        curr++;
        return true;
    }

    return false;
}

// match all tokens
bool PrismParser::remove_all( int (* callback)( char ) ){
    bool matched_one = false;

    while check( callback ) {
        matched_one = true;
    }

    return matched_one;
}




void PrismParser::require( int (* callback)( char ) ){
    if ( !check( callback ) )
        throw ParsingError( line, "-> " + get_token()  " <- " + "required token mismatch.\n" )
}

void PrismParser::require( char token ){
    if ( !check( token ) )
        throw ParsingError( line, "-> " + get_token()  " <- " + "required token mismatch.\n" )
}
