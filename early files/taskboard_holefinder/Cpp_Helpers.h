#pragma once

/***********  
Cpp_Helpers.h
James Watson , 2017 March
Shortcuts and Aliases , and Functions of general use for C++ programming

Template Version: 2017-05-26
***********/

#ifndef CPPHELPERS_H
#define CPPHELPERS_H

#if defined(unix) || defined(__unix__) || defined(__unix)
    #define IS_LINUX
#endif

#include <string> // --- string manipulation
#include <cstdio> // --- printf , getchar
#include <cmath> // ---- abs, min/max, trig, hyperbolic, power, exp, error, rounding
#include <limits> // --- Infinity
#include <stdlib.h> // - srand , rand , atof , strtof
#include <time.h> // --- time , for getting sys time and seeding random numbers
#include <ctime> // ---- time , for date formatting
#include <time.h> /* for clock_gettime */
#include <stdint.h> /* for uint64 definition */
#include <limits> // --- number limits of data types, limit on 'cin.ignore'
#include <ctype.h> // -- type tests
#include <cassert> // -- input/condition verification
//#define NDEBUG // ---- uncomment to disable assert()
#include <stdexcept> //- std errors

#include <vector> // --------- standard vector datatype , the friendly array } Data Structures
#include <list> // ----------- standard list datatype                       /
#include <map> // ------------ dictionaries                                /
#include <set> // ------------ sets                                       /
#include <algorithm> // ------ Searching structures , sort               /
#include <queue> // ---------- Priority Queue                           /
#include <utility> // -------- Pair , pair get , swap                  /
#include <initializer_list> // Pass array literals to funcs __________/

#include <iostream> // - standard input and output , istream } Input / Output
#include <fstream> // -- File I/O                           /
#include <sstream> // -- Text streams                      /
#include <sys/stat.h> // File status _____________________/

// Linux-Only Libraries
#ifdef IS_LINUX
#include <dirent.h> // Get directory info
#endif

// == Shortcuts and Aliases ==

// ~ Standard Shortcuts ~ // This is only for names that are unlikely to be shadowed
using std::cout; // ------- output to terminal
using std::cerr; // ------- Unbuffered terminal output , std::cerr automatically flushes all output as soon as it is written?
using std::clog; // ------- Unbuffered terminal output
using std::endl; // ------- newline
using std::cin; // -------- input from terminal
using std::ifstream; // --- File Input streams
using std::ofstream; // --- File Output streams
using std::ostream; // ---- Output streams
using std::stringstream; // String streams
using std::string; // ----- strings!           // Requires C++11
using std::to_string; // -- string conversion  // Requires C++11
using std::min; // -------- 'min' function
using std::max; // -------- 'max' function
using std::abs; // -------- Absolute value
using std::pow; // -------- Exponents
using std::ceil; // ------- Ceiling
using std::round; // ------ To nearest integer
using std::isnan; // ------ NaN Test
using std::isinf; // ------ Infinity Test
using std::printf; // ----- Our fave printing function from C
using std::swap; // ------- Swap 2 values , per CS 100
using std::sort; // ------- Get it sorted

// ~ Type Aliases ~ // Use this for long type names and names that are unlikley to be shadowed
using usll = unsigned long long; // big ints ( unsigned ) // Requires C++11
using llin = long long int; // ---- big ints              // Requires C++11
using uint = unsigned int; // ----- unsigned ints

// ~ Alias Templates ~ 
// URL: http://www.enseignement.polytechnique.fr/informatique/INF478/docs/Cpp/en/cpp/language/type_alias.html
template<class T> using stdvec = std::vector<T>;

// ~ Constants ~
#define EPSLNDB 1e-8d // ---------- Margin too small to care about , double
#define EPSLNFL 1e-4d // ---------- Margin too small to care about , float
#define BILLION 1000000000L // ---- ONE BILLION
double const INFTY_D   = std::numeric_limits<double>::infinity();
double const BILLION_D = 1e9;

// __ End Shortcuts __


// === Structs ===

// ~~ Indices ~~

struct IndexSearchResult{ // A container to hold a search result for an index that cannot have a negative value
    bool   result; // Is the result a valid one?
    size_t index; //- If so, which is the index we like best?
};

IndexSearchResult default_false_result();

struct IndexMultiResult{ // A container to hold a search result for indices that cannot have negative values
    bool /* -------- */ result; // Is the result a valid one?
    std::vector<size_t> indices; //- If so, which is the index we like best?
};

struct IndexDbblResult{ // A container to hold a search result for an index and some score for that index , No flag
    size_t index; // - Which is the index we like best?
    double measure; // How much do we like it?
    bool   result; //- Did we find it?
};

struct IndexFloatResult{ // A container to hold a search result for an index and some score for that index , No flag
    size_t index; // - Which is the index we like best?
    float  measure; // How much do we like it?
    bool   result; //- Did we find it?
};

struct IndexMatchResult{ // Container for the result of a search for a match and the indices representing the match
    bool   match; // Was there a match found?
    size_t sideA; // One half the match
    size_t sideB; // Other half of match
};

struct IndexMatches{ // A container to hold a search result for an index that cannot have a negative value
    size_t bestDex; // ------------ What is the best index?
    std::vector<size_t> indices; // Which indices are friends with 'bestDex'?
};

// ~~ ID Numbers ~~

struct IDSearchResult{ // A container to hold a search result for an ID that can be negative
    bool result; // Is the result a valid one?
    llin ID; // --- If so, which is the index we like best?
};

IDSearchResult default_ID_search_rs();

struct IDScoreLookup{ // A cheap, sequential "associative array" relating IDs to scores
    std::vector<llin>   IDvec; // -- Vector of ID numbers
    std::vector<double> scoreVec; // Scores associated with each ID
};

struct IDDbblResult{ // A container to hold a search result for an index that cannot have a negative value
    llin   ID; // ---- Which is the index we like best?
    double measure; // How much do we like it?
};

// ~~ Enums & Codes ~~

struct SuccessCode{
    bool   success; // Succeed or Fail?
    size_t code; // -- Status code
    string desc; // -- Description of { disposition , failure reason , results , etc }
};

struct BoolScore{ // Did we win?  By how much did we win or lose?
    bool   flag; //- Succeed or Fail?
    double score; // Scalar representation of how hard we are winning 
};

struct IndexSuccesLookup{ // A cheap, sequential "associative array" relating indices to scores / results
    bool /* ------------- */ result; // -- Did the search succeed or not?
    size_t /* ----------- */ bestDex; // - Winner , Winner , Chicken Dinner
    std::vector<SuccessCode> succesVec; // Vector of success reports
    std::vector<double>      scoreVec; //- Scores associated with each ID
    std::vector<size_t>      indices; // - Vector of indices, If left empty, indices are assumed to proceed from 0 to ( succesVec.size()-1 )
    std::vector<bool>        flagVec; // - Wildcard, vector of flags
};

// ___ End Struct ___


// == Struct Helpers ==



// __ End Helpers __


// == Debug Tools ==

void assert_report( bool assertion , string report ); // Reporting wrapper for 'assert'

void sep_dbg(); // Print a separator for debug information

void sep( string title = "" , size_t width = 6 , char dingbat = '=' ); // Print a separating title card for debug 

void newline(); //  print a new line

string yesno( bool condition );

void waitkey();
void waitkey( bool condition );
void waitkey( bool condition , string message );

// __ End Debug __


// == Logic / Control Tools ==

void toggle( bool& bit );

// __ End Logic / Ctrl __


// == Math Tools ==

void rand_init();

float rand_float(); // Note that this is not exactly uniformly distributed

double rand_dbbl();

bool dice_roll( double prob );

int randrange( int end );

size_t randrange( size_t end );

int randrange( int bgn , int end );

template<typename FLT>
FLT randrange( FLT lo , FLT hi ){ return (FLT) min( lo , hi ) + (FLT) rand_dbbl() * abs( hi - lo ); }

template<typename FLT>
std::vector<FLT> randrange_vec( FLT lo , FLT hi , size_t len ){
    std::vector<FLT> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( randrange( lo , hi ) );  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 ){ return ( (double) abs( op1 - op2 ) ) < EPSLNDB; }

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eqf( T op1 , T op2 ){ return ( (float) abs( op1 - op2 ) ) < EPSLNFL; }

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 , T eps ){ return ( abs( op1 - op2 ) ) < eps; }

template < typename T , typename U >
bool eq( T op1 , U op2 ){ return ( abs( (double)op1 - (double)op2 ) ) < EPSLNDB; }

usll tri_num( usll n );
size_t tri_num( size_t n );

double round_zero( double num );

template <typename T> 
int sign( T val ) { return ( T(0) < val ) - ( val < T(0) ); } // Return the sign if the number: -1 for val<0 , 0 for val==0 , 1 for val>0

template <typename T> 
bool is_even( T number ){  return ( number % (T)2 ) == 0;  }

template <typename T> 
bool is_odd( T number ){  return ( number % (T)2 ) != 0;  }

template <typename T> 
T clamp_val( T val , T minm , T maxm ){
    // NOTE: std::clamp is available since C++17 , https://en.cppreference.com/w/cpp/algorithm/clamp
    if( val < minm ) return minm;
    if( val > maxm ) return maxm;
    return val;
}

// __ End Math __


// == File Tools ==

bool file_exists( const string& fName );  // Return true if the file exists , otherwise return false

std::vector<string> readlines( string path ); // Return all the lines of text file as a string vector

void printlines( const std::vector<string>& lines ); // Print all the lines read from a file

string timestamp();

// = Linux File Tools =

#ifdef IS_LINUX

std::vector<string> list_dir( string dirStr ); // Return a string vector that contains all the entries in the directory

#endif

// _ End Linux _

// __ End File __


// == String Tools ==

void remove_all( string& rawStr , char keyChar ); // Destructively remove all instances of 'keyChar' from 'rawStr'

// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr );

string strip_after_dot( string fName ); // Return a copy of 'fName' with the first period and all following characters removed

string prepad(  string original , size_t totLen , char padChar = ' ' );
string postpad( string original , size_t totLen , char padChar = ' ' );

bool str_has_sub( string bigStr , string subStr ); // Return true if 'bigStr' contains 'subStr' , Otherwise return false

bool isnewline( char queryChar ); // Return true if the char is a newline , Otherwise return false

std::vector<double> tokenize_to_dbbl_w_separator( string rawStr , char separator ); // Return a vector of doubles between 'separator'
stdvec<float> tokenize_to_float_w_separator( string rawStr , char separator ); // Return a vector of floats between 'separator'

// __ End String __


// == Timing ==

class StopWatch{
    
public:
    
    StopWatch();
    StopWatch( double intervalSec );
    bool has_interval_elapsed();
    double seconds_elapsed();
    void mark();
        
protected:

    double interval;
    struct timespec tNow;
    struct timespec tMrk;
    
};

// __ End Timing __


// == Container Tools ==

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const std::vector<T>& vec ) { // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << vec[i];
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const std::vector<std::vector<T>>& vecVec ) { // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vecVec.size(); i++) {
        os << "  " << vecVec[i];
        if (i + 1 < vecVec.size()) { os << ", " << endl; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T1 , typename T2> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const std::vector<std::pair<T1,T2>>& vec ) { // ostream '<<' operator for pair vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << "(" << std::get<0>( vec[i] ) << " , " << std::get<1>( vec[i] ) << ")" ;
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const std::vector<std::pair<T,T>>& vec ) { // ostream '<<' operator for pair vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << "(" << std::get<0>( vec[i] ) << " , " << std::get<1>( vec[i] ) << ")" ;
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T>
size_t last_index( const std::vector<T>& vec ){  return vec.size() - 1;  }

template<typename T>
T last_elem( const std::vector<T>& vec ){  return vec[ vec.size() - 1 ];  } // NOTE: This function assumes there is at least one element

template<typename T>
std::vector<T> vec_copy( std::vector<T>& original ){ // NOTE: There is a const arg version as well
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
void extend_vec_with( std::vector<T>& original , const std::vector<T>& extension ){
    size_t xtLen = extension.size();
    for( size_t i = 0 ; i < xtLen ; i++ ){  original.push_back( extension[i] );  } // Using this so that you can extend vector with itself
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::vector<T> vec_join( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
    // Return the vector that is the concatenation of 'vec1' and 'vec2'
    std::vector<T> rtnVec;
    extend_vec_with( rtnVec , vec1 );
    extend_vec_with( rtnVec , vec2 );
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::vector<T> vec_plus_one( std::vector<T>& original , T plusOne ){
    std::vector<T> rtnVec;
    rtnVec = vec_copy( original );
    rtnVec.push_back( plusOne );
    return rtnVec;
}	

template<typename T> // NOTE: Templated functions must have their definition in the header file
void extend_vec_vec_with( std::vector<std::vector<T>>& original , std::vector<std::vector<T>>& extension ){
    size_t xtLen = extension.size();
    for( size_t i = 0 ; i < xtLen ; i++ ){  
        std::vector<T> temp = vec_copy( extension[i] );
        original.push_back( temp );  
    } // Using this so that you can extend vector with itself
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::vector<T> vec_range( T lo , T hi ){
    T i = 0;
    std::vector<T> rtnVec;
    if( lo == hi )
        rtnVec = { lo };
    else if( lo < hi )
        for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
    else
        for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
    return rtnVec;
}

std::vector<size_t> vec_index_zeros( size_t len );

std::vector<double> vec_dbbl_zeros( size_t len );

std::vector<std::vector<double>> vec_vec_dbbl_zeros( size_t len ); // Return a square vector of zeros

template<typename T> // NOTE: Templated functions must have their definition in the header file
T min_num_in_vec( std::vector<T> searchVec ){
    T      least   = searchVec[0]; 
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 0 ; i < numElem ; i++ ){ 
        least = min( searchVec[i] , least ); 
    } 
    return least;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T max_num_in_vec( const std::vector<T>& searchVec ){ // DEPRECATED , THOUGH USED IN ASP
    // NOTE: This function assumes that 'searchVec' has at least one element
    T      most    = searchVec[0]; 
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 1 ; i < numElem ; i++ ){ 
        most = max( searchVec[i] , most ); 
    } 
    return most;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T max( const std::vector<T>& searchVec ){ 
    // NOTE: This function assumes that 'searchVec' has at least one element
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    T      most    = searchVec[0]; 
    for( i = 1 ; i < numElem ; i++ ){ 
        most = max( searchVec[i] , most ); 
    } 
    return most;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t min_index_in_vec( std::vector<T> searchVec ){
    T      least   = searchVec[0]; 
    size_t minDex  = 0                ,
           i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 0 ; i < numElem ; i++ ){ 
        if( searchVec[i] < least ){
            least  = searchVec[i];
            minDex = i;
        }
    } 
    return minDex;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t max_index_in_vec( std::vector<T> searchVec ){
    T      most    = searchVec[0]; 
    size_t maxDex  = 0                ,
           i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 0 ; i < numElem ; i++ ){ 
        if( searchVec[i] > most ){
            most   = searchVec[i];
            maxDex = i;
        }
    } 
    return maxDex;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::list<T> lst_range( T lo , T hi ){
    T i = 0;
    std::list<T> rtnVec;
    if( lo == hi ){
        rtnVec.push_back( lo );
    }else if( lo < hi )
        for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
    else
        for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_list( T arg , std::list<T>& lst ){
    // Return true if 'arg' is in 'lst' , false otherwise
    return find( lst.begin() , lst.end() , arg ) != lst.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_vector( T arg , const std::vector<T>& vec ){
    // Return true if 'arg' is in 'st' , false otherwise
    // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
    // URL , const_iterator: https://stackoverflow.com/a/309589
    typename std::vector<T>::const_iterator it = find( vec.begin() , vec.end() , arg ); 
    return it != vec.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::vector<T> vec_intersection( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
    // Return a vector of all the common elements of 'vec1' and 'vec2'
    std::vector<T> rtnVec;
    size_t len = vec1.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( is_arg_in_vector( vec1[i] , vec2 ) ){  rtnVec.push_back( vec1[i] );  }  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_lessThan_in_vec( T query , std::vector<T>& searchVec ){
    // Return the number of elements of 'searchVec' that are less than 'query'
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( searchVec[i] < query ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_grtrThan_in_vec( T query , std::vector<T>& searchVec ){
    // Return the number of elements of 'searchVec' that are less than 'query'
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( searchVec[i] > query ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_elems_also_in( std::vector<T>& searchVec , std::vector<T>& compareVec ){
    // Return the number of elements of 'searchVec' that can be found in 'compareVec' (Repeats counted)
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( is_arg_in_vector( searchVec[i] , compareVec ) ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexSearchResult search_vec_for_arg( std::vector<T>& vec , T arg ){
    // Search the vector for the specified 'arg' and return the result
    // NOTE: This function assumes that the '==' comparison operator can be used on the vector items
    IndexSearchResult result = default_false_result();
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){
        if( arg == vec[i] ){  
            result.result = true;  
            result.index  = i;  
            return result; // Found a match, shortcut return with the result
        }
    }
    return result; // No match found , Return failed result
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexSearchResult search_vec_for_arg( const std::vector<T>& vec , T arg ){
    // Search the vector for the specified 'arg' and return the result
    // NOTE: This function assumes that the '==' comparison operator can be used on the vector items
    IndexSearchResult result = default_false_result();
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){
        if( arg == vec[i] ){  
            result.result = true;  
            result.index  = i;  
            return result; // Found a match, shortcut return with the result
        }
    }
    return result; // No match found , Return failed result
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec2_contains_vec1( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
    // Return true if every element of 'vec1' can be found in 'vec2' , Otherwise return false
    size_t len1 = vec1.size();
    IndexSearchResult result;
    for( size_t i = 0 ; i < len1 ; i++ ){ // For each of the elements of 'vec1'
        result = search_vec_for_arg( vec2 , vec1[i] );
        if( !result.result ){ return false; } // Mismatch , Shortcut to false
    }
    return true; // Of we got here, then none of the searches failed
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec_same_contents( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
    // Return true if and only if all of the elements of 'vec1' are in 'vec2' and vice-versa
    return vec2_contains_vec1( vec1 , vec2 ) && vec2_contains_vec1( vec2 , vec1 );
}

template<typename T>
T rand_choice( std::vector<T> searchVec ){ return searchVec[ randrange( searchVec.size() ) ]; }

std::vector<bool> bool_false_vector( size_t length );

std::vector<std::vector<bool>> bool_false_vec_vec( size_t length );

size_t random_false_elem_index( std::vector<bool> vec );

bool   all_elem_true( const std::vector<bool>& bulVec ); // - Return true if all elements true , otherwise return false
bool   any_elem_true( const std::vector<bool>& bulVec ); // - Return true if any elements true , otherwise return false
size_t count_elem_true( const std::vector<bool>& bulVec ); // Return the number of elements that are true

//  Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len' , Wraps forward and backwards , Python Style
llin indexw( llin len , llin i );

template<typename T>
size_t wrap_index_for_vec( size_t index , const std::vector<T>& vec ){  return index % vec.size();  }

size_t wrap_index_for_len( size_t index , size_t len );

std::ostream& operator<<( std::ostream& os , const std::set<int>& elemSet );

template<typename T>
std::vector<T> vec_copy( const std::vector<T>& original ){
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( std::vector<T>& original , T holdout ){ // non-const params
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , T holdout ){ // const params
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , const std::vector<T>& holdout ){
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_vector( original[i] , holdout ) ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , T holdElem , const std::vector<T>& holdVec ){
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  
        if(   ( !is_arg_in_vector( original[i] , holdVec ) )  &&  ( original[i] != holdElem )  ){  rtnVec.push_back( original[i] );  }  
    }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , const std::vector<std::vector<T>>& holdVec ){
    // vec_vec version // Would this be faster with a set?
    std::vector<T> rtnVec = vec_copy( original );
    size_t lenI = holdVec.size();
    // Successively remove elements based on the contents of each vec in 'holdVec'
    for( size_t i = 0 ; i < lenI ; i++ ){  rtnVec = vec_copy_without_elem( rtnVec , holdVec[i] );  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_one_index( const std::vector<T>& original , size_t index ){
    std::vector<T> rtnVec;
    if( index < original.size() ){  rtnVec.push_back( original[ index ] );  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_shuffled( const std::vector<T>& original ){
    std::vector<T> rtnVec = vec_copy( original );
    std::random_shuffle( rtnVec.begin() , rtnVec.end() );
    return rtnVec;
}

template<typename T>
void vec_assign_all_same( std::vector<T>& original , T sameVal ){
    // Assign each element to be the same value
    size_t len = original.size();
    for( size_t i = 0 ; i < len ; i++ ){  original[i] = sameVal;  }
}

template<typename T>
std::vector< size_t > vec_vec_len( std::vector<std::vector<T>>& vecVec ){
    std::vector<size_t> rtnVec;
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( vecVec[i].size() );  }
    return rtnVec;
}

template<typename T>
size_t vec_vec_longest_rep( std::vector<std::vector<T>>& vecVec ){
    // Return the length of the longest string representation of an element of a 2D std::vector
    size_t longest = 0;
    size_t len_i , len_j;
    string tempRep;
    len_i = vecVec.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = vecVec[i].size();
        for( size_t j = 0 ; j < len_j ; j++ ){
            tempRep = to_string( vecVec[i][j] );
            longest = max( tempRep.size() , longest );
        }
    }
    return longest;
}

template<typename T>
size_t vec_vec_longest_rep( const std::vector<std::vector<T>>& vecVec ){
    // Return the length of the longest string representation of an element of a 2D std::vector
    size_t longest = 0;
    size_t len_i , len_j;
    string tempRep;
    len_i = vecVec.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = vecVec[i].size();
        for( size_t j = 0 ; j < len_j ; j++ ){
            tempRep = to_string( vecVec[i][j] );
            longest = max( tempRep.size() , longest );
        }
    }
    return longest;
}

template<typename T>
void print_vec_vec( std::vector<std::vector<T>>& vecVec , size_t padLen = 1 ){
    // Return the length of the longest string representation of an element of a 2D std::vector
    size_t longest = vec_vec_longest_rep( vecVec );
    size_t len_i , len_j;
    string tempRep;
    len_i = vecVec.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = vecVec[i].size();
        for( size_t j = 0 ; j < len_j ; j++ ){
            tempRep = prepad( to_string( vecVec[i][j] ) , longest );
            cout << tempRep << string( padLen , ' ' );
        }
        cout << endl;
    }
}

template<typename T>
void print_vec_vec( const std::vector<std::vector<T>>& vecVec , size_t padLen = 1 ){
    // Return the length of the longest string representation of an element of a 2D std::vector
    size_t longest = vec_vec_longest_rep( vecVec );
    size_t len_i , len_j;
    string tempRep;
    len_i = vecVec.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = vecVec[i].size();
        for( size_t j = 0 ; j < len_j ; j++ ){
            tempRep = prepad( to_string( vecVec[i][j] ) , longest );
            cout << tempRep << string( padLen , ' ' );
        }
        cout << endl;
    }
}

template<typename T>
size_t vec_vec_any_empty( std::vector<std::vector<T>>& vecVec ){
    // Return true if any of the sub-vectors are empty, otherwise return false
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( vecVec[i].size() == 0 ){  return true;  }  }
    return false;
}

template<typename T>
std::vector<std::vector<T>> vec_vec_copy_nonempty( std::vector<std::vector<T>>& vecVec ){
    // Return a copy of 'vecVec' that contains only the non-empty sub-vectors
    size_t len    = vecVec.size() , 
           subLen = 0             ;
    std::vector<std::vector<T>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){  
        if( vecVec[i].size() > 0 ){
            std::vector<T> temp = vec_copy( vecVec[i] );
            rtnVecVec.push_back( temp );
        }  
    }
    return rtnVecVec;
}

template<typename T>
std::vector<std::vector<T>> vec_vec_copy( std::vector<std::vector<T>>& original ){
    size_t len_i , len_j;
    std::vector<std::vector<T>> rtnVecVec;
    len_i = original.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = original[i].size();
        std::vector<T> temp;
        for( size_t j = 0 ; j < len_j ; j++ ){
            temp.push_back( original[i][j] );
        }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

template<typename T>
std::vector<std::vector<T>> vec_vec_copy( const std::vector<std::vector<T>>& original ){
    size_t len_i , len_j;
    std::vector<std::vector<T>> rtnVecVec;
    len_i = original.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = original[i].size();
        std::vector<T> temp;
        for( size_t j = 0 ; j < len_j ; j++ ){
            temp.push_back( original[i][j] );
        }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

template<typename T>
void vec_vec_bottom_clear( std::vector<std::vector<T>>& vecVec ){
    // Clear the bottom-level vectors
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  vecVec[i].clear();  }
}

template<typename T>
void vec_vec_top_populate( std::vector<std::vector<T>>& vecVec , size_t N ){
    // Populate the top level with empty vectors
    vecVec.clear();
    for( size_t i = 0 ; i < N ; i++ ){
        std::vector<T> temp;
        vecVec.push_back( temp );
    }
}

template<typename T>
std::vector<std::vector<T>> row_vec_to_col_vec_vec( std::vector<T>& original ){
    // Convert a 1D standard vector into a standard 2D nested vector in which each vector has one element of the 'original'
    size_t len = original.size();
    std::vector<std::vector<T>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){
        std::vector<T> temp;
        temp.push_back( original[i] );
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

template<typename T>
T* elem_i_from_list( std::list<T>& searchList , size_t index ){
    // Access a list like it were a vector
    // NOTE: Retuning a pointer so that the same type can be used as both a return value and an error code
    typename std::list<T>::iterator it; // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
    size_t i     = 0                 ,
           len   = searchList.size() ;
    for( it = searchList.begin() ; it != searchList.end() ; ++it ){
        if( index == i ){  return &(*it);  }
        i++;
    }
    return nullptr; // ERROR , No such index!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexMultiResult first_in_common_btn_lists_index( std::list<T>& lst1 , std::list<T>& lst2 ){
    size_t i = 0 ,
           j = 0 ;
    typename std::list<T>::iterator it_i , // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
                                    it_j ;  
    IndexMultiResult result;
    result.result = false;
    for( it_i = lst1.begin() ; it_i != lst1.end() ; ++it_i ){
        for( it_j = lst2.begin() ; it_j != lst2.end() ; ++it_j ){
            result.result = *it_i == *it_j;
            if( result.result ){  break;  }
            j++;
        }
        if( result.result ){  break;  }
        i++;
    }
    result.indices.push_back( i );  result.indices.push_back( j );  // These indices point to the commonality, or the last indices
    return result;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexMultiResult first_in_common_btn_vec_index( std::vector<T>& lst1 , std::vector<T>& lst2 ){
    size_t i     = 0           ,
           j     = 0           ,
           len_i = lst1.size() , 
           len_j = lst2.size() ;
    
    IndexMultiResult result;
    for( i = 0 ; i < len_i ; i++ ){
        for( j = 0 ; j < len_j ; j++ ){
            result.result = lst1[i] == lst2[j];
            if( result.result ){  break;  }
        }
        if( result.result ){  break;  }
    }
    result.indices.push_back( i );  result.indices.push_back( j );  // These indices point to the commonality, or the last indices
    return result;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::vector<T> list_to_vec( std::list<T>& inputList ){
    // Return a list composed of all of the elements of 'inputList' , In order
    std::vector<T> rtnVec;
    typename std::list<T>::iterator it; // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
    for( it = inputList.begin() ; it != inputList.end() ; ++it ){
        rtnVec.push_back( *it );
    }
    return rtnVec;
}

template<typename T>
std::vector<T> linspace( T a , T b , size_t N ){
    // Original by Lorenzo Riano , URL: https://gist.github.com/lorenzoriano/5414671
    // NOTE: If N = 1 , A vector with only 'a' will be returned
    // NOTE: If N = 0 , An empty vector is returned
    T h = ( b - a ) / static_cast<T>( N - 1 );
    std::vector<T> xs( N );
    typename std::vector<T>::iterator x;
    T val;
    // This loop structure is unusual , will look at later
    for ( x = xs.begin() , val = a ; x != xs.end() ; ++x , val += h ){  *x = val;  }
    return xs;
}

template<typename T>
std::vector<T> operator*( const std::vector<T>& opVec , T opScl ){
    // Scale all of the elements of 'opVec' by 'opScl'
    std::vector<T> rtnVec;
    size_t len = opVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( opVec[i] * opScl );  }
    return rtnVec;
}

template<typename TFLT1 , typename TFLT2>
std::vector<TFLT1> operator*( const std::vector<TFLT1>& opVec , TFLT2 opScl ){  return opVec * (TFLT1)opScl;  }

template<typename T>
std::vector<T> operator-( const std::vector<T>& op1 , const std::vector<T>& op2 ){
    // Element-wise subtraction of op1 - op2
    std::vector<T> rtnVec;
    size_t len = op1.size();
    if( len == op2.size() ){
        for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( op1[i] - op2[i] );  }
        return rtnVec;
    }else{
        throw std::out_of_range ( "operator- std::vector<T> , Vector length mismatch: " + to_string( len ) + " and " + to_string( op2.size() ) );
    }
}

template<typename T>
std::vector<T> operator/( const std::vector<T>& op1 , const std::vector<T>& op2 ){
    // Element-wise division of op1 / op2
    std::vector<T> rtnVec;
    size_t len = op1.size();
    if( len == op2.size() ){
        for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( op1[i] / op2[i] );  }
        return rtnVec;
    }else{
        throw std::out_of_range ( "operator/ std::vector<T> , Vector length mismatch: " + to_string( len ) + " and " + to_string( op2.size() ) );
    }
}

template<typename T>
std::vector<T>& operator+=( std::vector<T>& opLeft , T opRght ){
    size_t len = opLeft.size();
    for( size_t i = 0 ; i < len ; i++ ){  opLeft[i] += opRght;  }
    return opLeft;
}

template<typename T>
std::vector<T>& operator+=( std::vector<T>& opLeft , std::vector<T>& opRght ){
    // Element-wise add-assign 'opLeft'
    // NOTE: Add-assign operations are only carried out through shared number of elements
    size_t len = min( opLeft.size() , opRght.size() );
    for( size_t i = 0 ; i < len ; i++ ){  opLeft[i] += opRght[i];  }
    return opLeft;
}

template<typename T>
std::vector<T> clamp_vec( std::vector<T>& raw , T lower , T upper ){
    // Clamp each value in 'raw' to within 'lower' and 'upper'
    stdvec<T> rtnVec;
    size_t    len = raw.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( clamp_val( raw[i] , lower , upper ) );  }
    return rtnVec;
}

template<typename T>
std::vector<T> abs( const std::vector<T>& op1 ){
    // Element-wise absolute value: abs( op1[i] ) 
    std::vector<T> rtnVec;
    size_t len = op1.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( abs( op1[i] ) );  }
    return rtnVec;
}

template< typename T , typename INT >
std::vector<T> subvec_of_from_to( const std::vector<T>& superVec , INT bgn , INT end ){
    size_t len = superVec.size();
    if(  ( bgn < len )  &&  ( end < len )  ){
        if( bgn > end ){  size_t swap = bgn;  bgn = end;  end = swap;  }
        // URL , Fetch sub-vector: https://stackoverflow.com/a/421615
        typename std::vector<T>::const_iterator it_bgn = superVec.begin() + bgn;
        typename std::vector<T>::const_iterator it_end = superVec.begin() + end + 1; // I do not prefer the Python slicing indices
        std::vector<T> rtnVec( it_bgn , it_end );
        return rtnVec;
    }else{
        throw std::out_of_range ( "subvec_of_from_to , An index was out of range , bgn: " + to_string( bgn ) + " , end: " + to_string( end )
                                + " out of " + to_string( len ) );
    }
}

template<typename T>
T most_numerous_value_in( const std::vector<T>& vec ){
    // Return the value that appears the most number of times in 
    // NOTE: If each value appears only once, then the first element will be returned
    size_t len           = vec.size() ,
           greatestCount = 0 , 
           elemCount     = 0 ;
    T      mostVal       = vec[0];
    for( size_t i = 0 ; i < len ; i++ ){
        elemCount = std::count( vec.begin() , vec.end() , vec[i] );
        if( elemCount > greatestCount ){
            greatestCount = elemCount;
            mostVal       = vec[i];
        }
    }
    return mostVal;
}

template<typename T>
std::vector<size_t> all_indices_equal_to_val( const std::vector<T>& vec , T val ){
    size_t len = vec.size();
    std::vector<size_t> matchDices;
    for( size_t i = 0 ; i < len ; i++ ){  if( vec[i] == val ){  matchDices.push_back( i );  }  }
    return matchDices;
}

// = Maps =

template < typename T1 , typename T2 >
std::vector<T1> map_keys_vec( std::map< T1 , T2 >& dict ){
    std::vector<T1>  rtnVec;
    typename std::map< T1 , T2 >::iterator it;
    for( it = dict.begin() ; it != dict.end() ; ++it ){  rtnVec.push_back( it->first );  }
    return rtnVec;
}

template < typename T1 , typename T2 >
std::vector<T1> map_has_key( const std::map< T1 , T2 >& dict , T1 key ){
    typename std::map< T1 , T2 >::iterator it = dict.find( key );
    return it != dict.end();
}

// _ End Maps _


// = Sets =

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_set( T arg , const std::set<T>& st ){
    // Return true if 'arg' is in 'st' , false otherwise
    // URL , C++ cannot recognize a templated typename
    typename std::set<T>::iterator it = st.find( arg );
    return it != st.end();
}

template<typename T> 
void set_insert_vec( std::set<T>& st , const std::vector<T>& vec ){
    // Insert all of the elements of 'vec' into 'st'.  'st' will automatically reject repeats
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  st.insert( vec[i] );  }
}

template<typename T> 
std::vector<T> set_to_vec( const std::set<T>& st ){
    // Return a vector that contains all of the elements in the set
    std::vector<T> rtnVec;
    for( typename std::set<T>::iterator it = st.begin() ; it != st.end() ; ++it ){  rtnVec.push_back( *it );  }
    return rtnVec;
}

template<typename T> 
std::vector<T> vec_minus_set( const std::vector<T>& vec , const std::set<T>& st ){ // This is an alias of the below , sorry
    // Return a vector of all the elements of 'vec' NOT in 'st'
    std::vector<T> rtnVec;
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_set( vec[i] , st ) ){  rtnVec.push_back( vec[i] );  }  }
    return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , const std::set<T>& holdout ){ // This is an alias of the above , sorry
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    std::vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_set( original[i] , holdout ) ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T> 
std::vector<T> vec_unique_only( const std::vector<T>& vec ){
    // Return a copy of 'vec' with only unique elements
    size_t len = vec.size();
    std::set<T> st;
    for( size_t i = 0 ; i < len ; i++ ){  st.insert( vec[i] );  }
    return set_to_vec( st );
}

// _ End Sets _

// = Queues =

// Empty the queue and discard all the values
template<typename T> // NOTE: Templated functions must have their definition in the header file
void erase_queue( std::queue<T>& Q ){    while( Q.size() > 0 ){  Q.pop();  }    }

template<typename T> // NOTE: Templated functions must have their definition in the header file
T queue_get_pop( std::queue<T>& Q ){ 
    // Get the front item, then pop it
    T temp = Q.front();
    Q.pop();
    return temp;
}

template<typename T> 
void enqueue_vec( std::queue<T>& Q , const std::vector<T>& additions ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  Q.push( additions[i] );  }
}

template<typename T> 
void enqueue_vec_not_in_set( std::queue<T>& Q , const std::vector<T>& additions , const std::set<T>& exclusions ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  
        if( !is_arg_in_set( additions[i] , exclusions ) ){  Q.push( additions[i] );  }
    }
}

template<typename T> 
void enqueue_vec_not_in_either_set( std::queue<T>& Q , const std::vector<T>& additions , const std::set<T>& ex1 , const std::set<T>& ex2 ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  
        if( 
            ( !is_arg_in_set( additions[i] , ex1 ) )
                &&
            ( !is_arg_in_set( additions[i] , ex2 ) )
        ){  Q.push( additions[i] );  }
    }
}

template< typename T , typename F > // NOTE: Templated functions must have their definition in the header file
T queue_get_pop( std::priority_queue< T , std::vector<T> , F >& Q ){ 
    // Get the front item, then pop it
    T temp = Q.top();
    Q.pop();
    return temp;
}

template< typename T , typename F > // NOTE: Templated functions must have their definition in the header file
void erase_queue( std::priority_queue< T , std::vector<T> , F >& Q ){    while( Q.size() > 0 ){  Q.pop();  }    }

// _ End Queue _

std::vector<double> err_vec( size_t len );

bool is_err( const std::vector<double>& vec ); // Return true if all of the elements are NaN, Otherwise return false

std::vector<std::vector<size_t>> enumerate_in_base( size_t digits , size_t base );

// __ End Container __


// === Functors ===

// == class Incrementer ==

class Incrementer{
public:
    Incrementer( llin start = 0 );
    llin operator()();
protected:
    llin count;
};

// __ End Incrementer __

// ___ End Functors ___


// === Memory Utils ===

string pointer_info_str( void* generalPointer );

template<typename T>
void delif( T*& ptr ){  
    // Delete the pointer if it contains data and set it to NULL
    if( ptr ){  
        delete ptr;  
        ptr = nullptr;
    }  
} 

template<typename T>
void clearif( std::vector<T*>& vec ){
    // Delete a vector of pointers, Then clear the vector
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  delif( vec[i] );  }
    vec.clear();
}

template<typename T>
void clearif( std::list<T*>& lst ){
    // Delete a list of pointers, Then clear the list
    typename std::list<T*>::iterator it;
    for( it = lst.begin() ; it != lst.end() ; ++it ){  delif( *it );  }
    lst.clear();
}

// ___ End Memory ___

#endif

/* == Useful Parts =========================================================================================================================

// ~~ Debug by Print ~~
bool SHOWDEBUG = true; // if( SHOWDEBUG ){  cout << "" << endl;  }

// ~~ cout << operator ~~
std::ostream& operator<<(std::ostream& os, const Gene& vec) {
    std::vector<float> codons = vec.copy_codons();
    os << "[ ";
    for (size_t i = 0; i < codons.size(); i++) {
        os << codons[i];
        if (i + 1 < codons.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}


// ~~ Function Object (functor) ~~
struct myclass {
  bool operator() (int i,int j) { return (i<j);}
} myobject;


// ~~ Array Initialization ~~ 
int foo[3][2] = { { 1 , 2 } , { 3 , 4 } , { 5 , 6 } }; // Nested array assignment test


// ~ Function Pointer ~
void (*foo)(int) // Declare function pointer 'foo' to a function that takes one int and returns void


// ~ Lambda (C++11) ~
https://www.cprogramming.com/c++11/c++11-lambda-closures.html

// LAMBDA: [&] Capture all named variables by reference
// Type defaults to void if there is no return statement
auto checkAndUpdateBest = [&]( Eigen::Vector3d direction ){
    result = query_NDBG( NDBG , movedParts , referenceParts , direction );
    if( result.result ){
        currAng = angle_between( direction , avrgDir );
        if( currAng < bestAng ){
            bestDir = direction;
            bestAng = currAng;
        }
    }
}; // NOTE: Lambda expression must end with a semicolon

bool SHOWDEBUG = true  , // if( SHOWDEBUG ){  cout << "" << endl;  }
     BREAKPNTS = false ; // if( BREAKPNTS ){  waitkey( CONDITION , "MSG" );  }

  __ End Parts __________________________________________________________________________________________________________________________ */
  
  
/* == Spare Parts ==========================================================================================================================

template<typename T>
size_t vec_wrap_index( std::vector<T> vec , llin rawIndex ){
    size_t vecLen = vec.size();
    if( rawIndex >= 0 ){
        return rawIndex % vecLen;
    } else {
        
    }
}

 __ End Parts ___________________________________________________________________________________________________________________________ */
