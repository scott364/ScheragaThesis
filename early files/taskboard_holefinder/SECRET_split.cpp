#include <string>
using namespace std;
/* ~ Note ~
   The split() function is provided, and functions as it does in the homework 
   assignments, to make parsing the output easier. 
   Recall that split takes a string `s` and splits it at the input delimiter `sep`, 
   to fill the array `words[]` up to a capacity of `max_words`. The return value of 
   split is the number of actual words the string is split into.
   
   ~ Function Signature ~
   int split( string s , char sep , string words[] , int max_words )  */

int split( string s , char sep , string words[] , int max_words ){
    int    arrDex = 0 , // ------- Current index to insert a new word
           len    = s.length(); // Length of input string
    string currWord; // ---------- Current word between separators
    s += sep; // Delimiter termination hack
    for( int i = 0 ; i < len ; i++ ){
        if( s[i] != sep ){ // If not separator, accumulate char to `currWord`
            currWord += s[i];
        }else{ // else is separator, 
            // add word if we accumulated one and there is space for it
            if( currWord.length() > 0 && arrDex < max_words ){
                words[ arrDex ] = currWord; // Assign word to array
                arrDex++; // Increment index so that next word is in correct spot
            }
            currWord = "";
        }
    }
    /* If words were found, then `arrDex` is always one more than the last 
       index populated, else is zero. Thus we can use it for a word count! */
    return arrDex;
}