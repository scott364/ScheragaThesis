#include <iostream>
#include <fstream>
using namespace std;

// Split will not be needed for Practicum 2, 
// but you can define it in "SECRET_split.cpp" to make this example run
#include "SECRET_split.cpp" // Comment out if "SECRET_split.cpp" NOT available!
#define SPLIT_AVAIL  // ______ Comment out if "SECRET_split.cpp" NOT available!
// Advanced Topic: COMPILE CONDITIONALLY - http://www.cplusplus.com/doc/tutorial/preprocessor/
/* ~ Note ~
   The split() function is provided, and functions as it does in the homework 
   assignments, to make parsing the output easier. 
   Recall that split takes a string `s` and splits it at the input delimiter `sep`, 
   to fill the array `words[]` up to a capacity of `max_words`. The return value of 
   split is the number of actual words the string is split into.
   
   ~ Function Signature ~
   int split( string s , char sep , string words[] , int max_words )  */

using namespace std;

// ~~~ TOPIC 1. Write a simple function with input checking ~~~

double cuboid_volume( double l , double w , double h ){
    /* Return the volume of the cuboid if all dimensions are 
       greater than or equal to zero, otherwise return -1.0 */
    
    if(  l < 0.0  ||  w < 0.0  ||  h < 0.0  ) // Incorrect argument check
        return -1.0;

    // If we made it here, then `return` was not executed, else not required
    return l * w * h;
}

// ~~~ TOPIC 2. Operate on arrays passed to function ~~~

int linear_search_max_min( double arr[] , int len , bool mx ){
    // Return the index of the max value if `mx` is true
    // Return the index of the min value if `mx` is false
    // Return -1 if array has no elements
    
    if( len <= 0 )  return -1; // Arg check

    // If we're here, Array has at least one element, start at elem 0
    double extremeVal = arr[0];
    int    rtnIndex   = 0;

    // Begin for-loop with second elem if it exists, otherwise loop exits
    for( int i = 1 ; i < len ; i++ ){
        
        // If user asked for max and current elem larger than extreme
        if( mx && arr[i] > extremeVal ){
            extremeVal = arr[i];
            rtnIndex   = i;
        }

        // If user did not ask for max and current elem lesser than extreme
        if( !mx && arr[i] < extremeVal ){
            extremeVal = arr[i];
            rtnIndex   = i;
        }
    }
    // For-loop exit, Search for max/min complete, return most extreme index
    return rtnIndex;
}

// ~~~ TOPIC 3. Print a series ~~~

int fibonacci( int n ){
    /* Print the first `n` fibonacci numbers surrounded by curly braces and with 
       spaces surrounding each number.  Sequence is followed by newline. 
       Return the nth number in n >= 1 else return -1 */
    if( n < 1 )  return -1;
    
    int twoBefore = 0;
    int oneBefore = 1;
    int rtnFib    = 0;
    
    cout << "{ "; // Open brace and print space
    for( int i = 1 ; i <= n ; i++ ){
        // First fib is 0, else
        if( i > 1 ){
            // Caculate fibonacci
            rtnFib = twoBefore + oneBefore;
        }
        // print ith fib with space
        cout << rtnFib << " ";
        // Update previous in sequence
        twoBefore = oneBefore;
        oneBefore = rtnFib;
    }
    cout << "}" << endl; // Close brace and emit newline
}

// ~~~ TOPIC 4. Read a file and process contents ~~~

/* Task is to read a file with each line in the format
Person Name, 10, 9, 14, 18, 7
Where each non-empty line begins with a name and is followed by 1 to 20 integer 
values.  You must print each name, followed by a comma and space, with each
odd value on the line followed by a comma and space, ending with a newline.  
Person Name, 9, 7,
Ignore empty lines in the file
If the file cannot be opened, print "File open FAILED!\n"
 */
#ifdef SPLIT_AVAIL 

void print_odds( string fName ){
    ifstream txtFile; // --------- File handle
    string   line; // ------------ Most recent line of text read
    int      maxWords = 21 , // -- Max number of elements `split` will find
             N_found  =  0 , // -- Number of elements actually found
             num      =  0 ; // -- Current number
    string   words[ maxWords ]; // Populated by `split`
    
    // Open the file
    txtFile.open( fName );

    // If the file open did NOT fail, we have work to do
    if( !txtFile.fail() ){ 

        while( getline( txtFile , line ) ){ // While there is a line to read

            for(int i =0; i<maxWords  ; i++){  words[i]="";  }  // Clear `words`
            // Otherwise will contain words from a previous, longer line

            // If NOT empty line, then print
            if( line.length() > 1 ){ // line may sometimes contain a newline ONLY (1 char)
                // Split the line
                N_found = split( line , ',' , words , maxWords );
                // Print name
                cout << words[0] << ", ";
                // Iterate over remaining elements up to number found
                for( int i = 1 ; i < N_found ; i++ ){
                    num = stoi( words[i] ); // String to int conversion
                    if( num % 2 == 1 )
                        cout << num << ", ";
                }
                // End with newline whether there were odds or not
                cout << endl;
            }
        }
        // CLOSE the file
        txtFile.close();
    }else{
        cout << "File open FAILED!\n";
    }
}
#endif

int main(){  
    cout << endl;
    
    // ~~~ TOPIC 1. Write a simple function with input checking ~~~
    cout << "cuboid_volume: " << cuboid_volume( -1.0 , 200   , 300   ) << endl;
    cout << "cuboid_volume: " << cuboid_volume(  1.2 ,   2.3 ,   3.4 ) << endl << endl;

    // ~~~ TOPIC 2. Operate on arrays passed to function ~~~
    double arr[10] = { -10.39, -0.33, 0.0, -13.64, 11.54, 5.61, 9.49, 16.56, 
                         5.66, -13.73 };
    int index = 0;
    index = linear_search_max_min( arr , 10 , true );
    cout << "Maximum: " << arr[ index ] << ", at index: " << index << endl;
    index = linear_search_max_min( arr , 10 , false );
    cout << "Minimum: " << arr[ index ] << ", at index: " << index << endl << endl;

    // ~~~ TOPIC 3. Print a series ~~~
    cout << "First 3 fibonacci: "; 
    fibonacci( 3 );
    cout << "First 9 fibonacci: "; 
    fibonacci( 9 );
    cout << endl;

    // ~~~ TOPIC 4. Read a file and process contents ~~~
    #ifdef SPLIT_AVAIL
    print_odds( "example.txt" );
    cout << endl;
    #endif

    // ~~~ TOPIC 5. Writing to Files ~~~
    ofstream fileHandle;
    fileHandle.open("myFile.txt");
    int j = 0;
    string str = "Hello, file!";
    while( j < 4 ){
        fileHandle << str 
                   << endl; // Note that we can output `endl` to any stream!
        j++;
        cout << str << endl;
    }
    fileHandle.close();
}
