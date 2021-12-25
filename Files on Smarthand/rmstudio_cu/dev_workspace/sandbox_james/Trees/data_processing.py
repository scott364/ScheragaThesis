########## INIT ###################################################################################

##### Imports #####

### Standard ###
import os, csv, operator, traceback, sys, shutil
from math import ceil
import os.path
from dateutil.parser import parse

### Special ###
import xml.etree.ElementTree as ET
import xml.parsers.expat as EX
import matplotlib.pyplot as plt
import numpy as np
import scipy
from scipy import stats
import matplotlib.pyplot as plt
import torch
from sklearn.linear_model import LinearRegression

##### Constants #####
def init():
    """ Export globals """
    # the only global variables Python really has are module-scoped variables, https://stackoverflow.com/a/1978076
    global _ORIGINAL_DATA, _EXPORT_DIR, _DATA_DIR
    _ORIGINAL_DATA = "/home/jwatson/rss_data/"
    _EXPORT_DIR    = "/home/jwatson/RAL_data/"
    _DATA_DIR      = _EXPORT_DIR

init()


########## Utilities ##############################################################################


def get_progress_bar( div = 25, char = '>', between = ' ', counter = 0 ):
    """ Enclose a counter and return a function that emits a char every `div` calls """
    
    def progchar():
        """ Emit progress character ever `div` iter """
        nonlocal div, char, between, counter
        counter += 1
        if counter % div == 0:
            print( char , end = between , flush = 1 )
            return True
        else:
            return False
    
    return progchar


########## File Operations ########################################################################


def ensure_dir( dirName , gracefulErr = 1 ):
    """ Create the directory if it does not exist """
    if not os.path.exists( dirName ):
        try:
            os.makedirs( dirName )
        except Exception as err:
            if gracefulErr:
                print( "ensure_dir: Could not create" , dirName , '\n' , err )
            else:
                raise IOError( "ensure_dir: Could not create" + str( dirName ) + '\n' + str( err ) )
                
                
def npy_to_array( path, dtype = 'float' ):
    """ Return the NPY file as an array """
    return np.array( np.load( path ) ).astype( dtype = dtype )


def listpaths( srcDir ):
    """ Return all the names in `srcDir` as full paths """
    rtnPaths = []
    srcNames = os.listdir( srcDir )
    for name in srcNames:
        rtnPaths.append( os.path.join( srcDir, name ) )
    return rtnPaths


def get_file_EXT( pathOrName ):
    """ Return capitalized file extension of `pathOrName`, without the '.' """
    # NOTE: This function assumes that the characters that follow the last '.' are the file extension
    try:
        return str( pathOrName ).split('.')[-1].upper()
    except Exception:
        return ""

    
def filter_dir_by_EXT( srcDir, EXT ):
    """ Return paths to files in toplevel `srcDir` that match extension `EXT` """
    EXT = str( EXT ).upper()
    res = os.listdir(srcDir)
    res = [item for item in res if os.path.isfile( os.path.join( srcDir, item ) )]
    res = [os.path.join(srcDir,item) for item in res if get_file_EXT(item)==EXT]
    return res


def cp_mv_files_w_EXT( srcDir, dstDir, EXT, move=0 ):
    """ Copy all the files in toplevel `srcDir` that match extension `EXT` to `dstDir` """
    srcMatch = filter_dir_by_EXT( srcDir, EXT )
    moved    = []
    faild    = []
    for matchPath in srcMatch:
        fName   = str( matchPath ).split('/')[-1]
        srcPath = os.path.join( srcDir, fName )
        dstPath = os.path.join( dstDir, fName )
        print( "Attempt:", srcPath, os.path.isfile(srcPath),  "--to->", dstDir, end=": " )
        try:
            if move:
                shutil.move( srcPath, dstDir )
            else:
                shutil.copy( srcPath, dstDir )
            moved.append( dstPath )
            print( "OK" )
        except Exception as ex:
            faild.append( dstPath )
            print( ex )
    return {
        'moved'  : moved, 
        'failed' : faild,
    }


def change_filename_EXT( name_or_path, nuEXT, suppressPath = 1 ):
    """ Return a version of `name_or_path` so that that it has a `nuEXT` """
    parts = str( name_or_path ).split('/')
    if len( parts ) > 1:
        name    = parts[-1]
        path    = parts[:-1]
        wasPath = 1
    else:
        name    = parts[0]
        path    = ""
        wasPath = 0
    
    nuName = name.split('.')[0] + '.' + nuEXT
    
    if suppressPath or (not wasPath):
        return nuName
    else:
        return os.path.join( *path, nuName )


                
########## Recording Retrieval ####################################################################


def xml_to_csv_file( inPath, outDir ):
    """ Convert awkward XML files to usable CSV """
    
    print( "Converting" , inPath , ", Exists?:" , os.path.isfile( inPath ) , ", Begin ..." )
    
    #  0. Parse the XML
    try:
        with open( inPath, 'r' ) as xml_file:
            tree = ET.parse( xml_file )
        root = tree.getroot()
    except ET.ParseError as err:
        raise RuntimeError( "Could not parse this string due to error:\n\tCode: " + \
                            EX.ErrorString( err.code ) + " at " + str( err.position ) )
    
    #  1. Extract the label
    if root.find('label').text == '1':
        failorsucc = 'Success'
    else:
        failorsucc = 'Fail'
        
    #  2. Fetch XML data
    xmlData = root.find('data').findall('item')
    dataLen = len( xmlData )
    print( "There are" , dataLen , "data points in this file." )
    
    # NOTE: `xml` retrieves children in an unordered fashion.  Therefore, we must sort by date.  The steps are
    #       1. Read unordered XML 
    #       2. Sort
    #       3. Write to CSV
    
    timeSeries = []
    #  3. For every every record, write a row to the temp CSV
    for c, member in enumerate( xmlData ):
        row  = []
        
        #  4. Fetch the timestamp and convert to ms since first datapoint
        time = parse(member.findall('item')[0].text)
        if c==0:
            firsttime = time
        time_ms = float( (time-firsttime).total_seconds()*1000 )
        row.append( time_ms )
        
        #  5. Fetch each element of the FT vector
        for i in range(6):
            ft_i = float( member.findall('item')[1].findall('item')[i].text )
            row.append( ft_i )
            
        #  6. Store the datapoint as a row in the timeseries
        timeSeries.append( row )
        
    #  7. Define a timestamp comparison for sorting rows of data
    func = operator.itemgetter(0)
    def floatGetter(s):
        return float(func(s))
    
    #  8. Sort data (THIS PROBABLY DOES NOTHING?)
    sorted_FT_data = sorted( timeSeries, key = floatGetter )
    
    #  9. Open the output file for writing and create a writer
    filename   = os.path.split( inPath )[-1]
    newcsvPath = os.path.join( outDir , filename.split('.')[0]+'_'+failorsucc+'_RAW.csv' ) 
    outputFile = open( newcsvPath , 'w' )
    csvwriter  = csv.writer( outputFile )
    
    # 11. Write all rows to CSV and close file
    print( "About to write to" , newcsvPath , "..." )
    tenPcnt = int( dataLen / 10.0 )
    for i, datum in enumerate( sorted_FT_data ):
        csvwriter.writerow( datum )
        if i % tenPcnt == 0:
            print( '>', end = ' ' , flush = True )
    print( "100%" )
    outputFile.close()
    print( "COMPLETE!\n" )
    
    # N. Return the path to the new CSV
    return newcsvPath
    
    
def xml_to_csv_dir( inDir , outDir ):
    """ Convert all XML in a directory to CSV """
    
    # 0. Check that input and output directories exist
    p_in_exist  = os.path.isdir( inDir  )
    ensure_dir( outDir , gracefulErr = 1 )
    p_out_exist = os.path.isdir( outDir )
    
    print( "The input  directory is" , inDir  , "Exists?:" , p_in_exist  )
    print( "The output directory is" , outDir , "Exists?:" , p_out_exist )
    print( "# BEGIN #" )
    
    if not (p_in_exist and p_out_exist):
        print( "Cannot process files, a directory is missing!" )
        return None
    
    # 1. Fetch all file names
#     inFiles = os.listdir( inDir )
    inFiles = filter_dir_by_EXT( inDir, 'xml' )
    print( "There are" , len( inFiles ) , "files to process in this directory\n" )
    
    csvPaths = []
    # 2. For every file in the directory
    for fName in inFiles:
        fPath = os.path.join( inDir , fName )
        print( "Full path:" , fPath , "about to process...\n" )
        
        # 3. Attempt to convert
        try:
            csvPaths.append(  xml_to_csv_file( fPath, outDir )  )
        except Exception as err:
            print( fName , "FAILED with error:" , err )
        print()
            
    # N. Return the names of the successful files only
    return csvPaths
    
    
    
########## Data Retrieval #########################################################################


def get_label_from_filename( fName , passMatch , failMatch , PASS = 1.0 , FAIL = 0.0 ):
    """ Return pass/fail based on auto-generated file names """
    if passMatch in fName:
        return PASS
    elif failMatch in fName:
        return FAIL
    else:
        raise ValueError( "get_label_from_filename: " + \
                          str( namSlc ) + " did not match " + str( passMatch ) + " or " + str( failMatch ) )


def load_CSV_for_NN( csvPath , elemParser = float ):
    """ Load a CSV file into a multidim array """
    rtnArr = []
    with open( csvPath , 'r' ) as fCSV:
        reader = csv.reader( fCSV )
        for row in reader:
            datum = [ elemParser( elem ) for elem in row ]
            rtnArr.append( datum )
    return np.array( rtnArr )



########## Data Processing ########################################################################
# NOTE: It does not make sense to normalize each column on its own because relative magnitudes between channels are NOT PRESERVED


def normalize_matrix( data_np, loBound = -1.0, hiBound = 1.0 ):
    """ Rescale `data_np` from [<global min>,<global max>] to [`loBound`,`hiBound`] """
    # 1. Get the greatest and least element in matrix
    hi = np.amax( data_np )
    lo = np.amin( data_np )
    # 2. Make same-size matrices of the limits
    hiArr = np.full( data_np.shape, hi      )
    loArr = np.full( data_np.shape, lo      )
    loBAr = np.full( data_np.shape, loBound )
    # 3. Scale the entire matrix and return
    #      (set min to zero) / (scale to [0,1])* (scale to [loB,hiB] ) + (start at loB)
    return (data_np - loArr) / (hiArr - loArr) * ( hiBound - loBound ) + loBAr


def normalize_rows_as_probs( data_np ):
    """ Normalize every row to sum to 1.0 """
    return scipy.special.softmax( data_np, axis = 1 )
    


def normalize_column_groups( data_np, groups, loBound = -1.0, hiBound = 1.0 ):
    """ Normalize individual groups of columns [..., [bgn,end], ...], For use in dataset with several channels of the same type """
    # 0. Copy the input matrix in case there are columns the user does not wish to normalize
    rtnArr = data_np.copy()
    # 1. For each group, run the matrix-wide normalization procedure
    for group in groups:
        # A. Get the group column index bounds
        gBgn = group[0]
        gEnd = group[1]
        # B. Fetch columns, normalize, and assign back to return matrix
        rtnArr[ : , gBgn:gEnd ] = normalize_matrix(  data_np[ : , gBgn:gEnd ], loBound, hiBound  )
    return rtnArr
    

def normalize_dataset_and_save( csvPath, npyPath, groups, loBound = -1.0, hiBound = 1.0 ):
    """ Load data from CSV, Noramlize it, then save it to NPY """
    
    # 1. Load the data
    data_NP = load_CSV_for_NN( csvPath , elemParser = float )
    
    # 2. Normalize data
    norm_NP = normalize_column_groups( data_NP, groups, loBound, hiBound )
    
    # 3. Save data
    np.save( npyPath, norm_NP, allow_pickle = 0 )

    
def normalize_directory_and_save( csvDir, npyDir, groups, loBound = -1.0, hiBound = 1.0 ):
    """ Load data from CSV directory, Noramlize it, then save it to NPY directory """
#     csvNames = os.listdir( csvDir )
    csvNames = filter_dir_by_EXT( csvDir, 'csv' )
    
    for csvNam in csvNames:
        print( "# Converting" , csvNam, "... #" )
        npyNam = change_filename_EXT( csvNam, 'npy' )
        normalize_dataset_and_save( 
            os.path.join( csvDir, csvNam ) , 
            os.path.join( npyDir, npyNam ) , 
            groups, loBound, hiBound
        )
        print( "Done\n" )
    
    
    
########## Time Series Interpolation ##############################################################


### Double Exp. Window Filter and Interpolate ###


def Holt_Winters_interpolate( data_np, alpha_dataFactor, beta_slopeFactor, ts = 20.0, timeCol = 0 ):
    """ Simultaneously apply Timestep-aware double exponential window and  """
    # https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
    # https://www.itl.nist.gov/div898/handbook/pmc/section4/pmc434.htm
    # https://www.nist.gov/nist-research-library/reference-format-nist-publications
    # NOTE: Returned array will have the same shape as the input array, see below
    
    # 0. Get dataset info
    tC     = timeCol # ---------- Column holding timestamps
    dRow   = data_np.shape[0] # ------- Number of datapoints in the 
    dCol   = data_np.shape[1] # ------- Number of channels
    lastTS = data_np[ -1, tC ] # ------ Last timestamp of the recording
    
    # 1. Split into timestamps and data
    Tx   = data_np[ : , tC    ].flatten()
    X    = data_np[ : , tC+1: ]
    xRow = X.shape[0]
    xCol = X.shape[1]
    
    # 2. Allocate and pre-populate the return series
    rRow = int( lastTS/ts )+1 # Number of `dt` timesteps that will fit in the recording == number of rows of returned data
    rCol = dCol
    R    = np.zeros( ( rRow, rCol ) )
    dtr  = ts
    R[0,:] = data_np[0,:]
    
    # Abbreviate eq. params
    A = alpha_dataFactor
    B = beta_slopeFactor
    
    # Raw X
    tx_i  = Tx[0]
    x_i   = X[0,:]
    dtx_i = Tx[1] - Tx[0]
    
    # Interpolated R
    j    = 0
    tr_j = 0.0
    sr_j = X[0,:]
    br_j = ( X[1,:] - X[0,:] ) / dtx_i
    
    
    # 3. For every row in the original data, Calculate the rolling exponential
    for i in range( 1, xRow ):
        
        # A. Stash X_{i-1} params
        tx_im1  = tx_i
        x_im1   = x_i
        dtx_im1 = dtx_i
        
        # A. Update X_{i} params
        tx_i  = Tx[i]
        x_i   = X[i,:]
        dtx_i = tx_i - tx_im1
        mx_i  = (x_i - x_im1)/dtx_i
        
        # Case 1: tr is behind tx, Update the R estimates and write rows
        while tr_j < tx_i:
            
            # 1. Stash R_{j-1} params
            tr_jm1 = tr_j
            sr_jm1 = sr_j
            br_jm1 = br_j
            
            # 2. Update R_{j} params
            j += 1
            tr_j = j*dtr
            
            # 3. Apply estimates of x_i until tx no longer applies
            # Smoothed data
            fctr = dtr/dtx_im1 if (dtr < dtx_im1) else dtx_im1/dtr
            sr_j = A*x_i + (1-A)*(sr_j + br_jm1*fctr) 
            # Smoothed slope
            fctr = dtr/dtx_i if (dtr < dtx_i) else dtx_i/dtr
            br_j = B*mx_i*fctr + (1-B)*br_jm1 
            
            # 4. Store the smoothed datapoint (if there is room)
            if j < rRow:
                R[ j, tC    ] = tr_j
                R[ j, tC+1: ] = sr_j
            
        # Case 2: tx is behind tr, Do nothing            

    # Make sure that last datapoint makes it in
    R[-1,:] = data_np[-1,:]
        
    # N. Return
    return R


def dbbl_exp_interpolate_directory_and_save( inputDir, outptDir, alpha, beta, ts = 20.0, timeCol = 0 ):
    """ Load data from CSV directory, Noramlize it, then save it to NPY directory """
    
    # 0. Fetch all file names
#     inpNames = os.listdir( inputDir )
    inpNames = filter_dir_by_EXT( inputDir, 'npy' )
    
    # 1. For every name in the input directory, build paths and interpolate to a standard uniform `dt`
    for inpNam in inpNames:
        print( "# Interpolate" , inpNam, "with dt =" , ts, end = "..." )
        # A. Build paths
        parts   = inpNam.split('.')
        outNam  = str(parts[0]).replace( "_RAW", "_INTR" ) + '.' + str(parts[-1])
        print( str(parts[0]).replace( "_RAW", "_INTR" ) , '.' , str(parts[-1]) )
        inpPath = os.path.join( inputDir, inpNam )
        outPath = os.path.join( outptDir, outNam )
        # B. Load file
        inpSeries = npy_to_array( inpPath )
        # C. Interpolate
        outSeries = Holt_Winters_interpolate( inpSeries, alpha, beta, ts, timeCol )
        # D. Save
        np.save( outPath, outSeries, allow_pickle = 0 )
        print( "Done!" )


##### BROKEN INTERPOLATION ######################

        
def intmax( a, b ):
    """ int(max( a, b )) """
    return int(max( a, b ))


def intmin( a, b ):
    """ int(min( a, b )) """
    return int(min( a, b ))


def slope_intercept_2d( p1, p2 ):
    """ Return the slope and intercept for a line passing through two 2D points """
    m = ( p2[1] - p1[1] ) / ( p2[0] - p1[0] ) # Slope = rise / run
    b = p1[1] - p1[0] * m # ------------------- Y-Intercept
    return [m, b]


def interpolate_series( X, dt, timeCol = 0 ):
    """ Interpolate N-channel data "Recorded" `X` into equal `dt` spaced datapoints in "Returned" dataset """
    
    # FIXME: THIS IS INCORRECT, MAKE THE INDEX COUNTING LIKE THE EXPONENTIAL INTERPOLATION
    
    # Recorded --in-->
    # Returned <-out--
    _DEBUG   = 1
    progchar = get_progress_bar( div = 25 )
    
    # 0. Get dataset info
    tC     = timeCol # ---------- Column holding timestamps
    xRow   = X.shape[0] # ------- Number of datapoints in the 
    mCol   = X.shape[1] # ------- Number of channels
    lastTS = X[ -1, tC ] # ------ Last timestamp of the recording
    rRow   = int( lastTS/dt )+1 # Number of `dt` timesteps that will fit in the recording == number of rows of returned data
    
    # 1. Pre-allocate array  &&  Load known data  &&  Prediction Model
    R = np.zeros( (rRow, mCol) ) # ------------ We know the required shape
    R[0,: ] = X[0,:] # ------------------------ No matter what, the beginning of the recording equals the beginning of the returned series
    R[:,tC] = [ i*dt for i in range( rRow ) ] # Populate timestamps
#     modl    = LinearRegression()
    
    # 2. Init bookkeeping
    i_bgn_x = 0 # Beginning of segment to fit <-------- Recorded
    i_end_x = 1 # End _____ of segment to fit 
    i_bgn_r = 0 # Beginning of segment to interpolate < Returned
    i_end_r = 1 # End _____ of segment to interpolate
    
    
    # 3. While there are rows of the output matrix to process
    while i_end_r < rRow-1:
        # Loop begins with a 1-length slice on both the recorded and returned side
    
        # A. Get the timesteps corresponding to the endpoints
        nxtTSx = X[ i_end_x, tC ]
        nxtTSr = i_end_r * dt
        
        ## B. Handle timestamp mismatch cases ##
        
        # Case 1: The recorded step extends farther than the returned step
        if nxtTSx > nxtTSr:
            # Step the returned endpoint until the last timestep before the next recorded timestep
            while nxtTSx > nxtTSr and (i_end_r < rRow-1):
                i_end_r += 1
                nxtTSr = i_end_r * dt
                
            if nxtTSx < nxtTSr:
                i_end_r -= 1
                nxtTSr = i_end_r * dt
        
        # Case 2: The returned step extends farther than the recorded step
        elif nxtTSx < nxtTSr:
            # Step the recorded endpoint until the last timestep before the next returned timestep
            while (nxtTSx < nxtTSr) and (i_end_x < xRow-1):
                i_end_x += 1
                nxtTSx = X[ i_end_x, tC ]
                
            if nxtTSx > nxtTSr:
                i_end_x -= 1
                nxtTSx = X[ i_end_x, tC ]
        
        # Case 3: Both timesteps occur at precisely the same time, No adjustment needed
        
        if _DEBUG and progchar():
            print( (i_bgn_x, i_end_x), (i_bgn_r, i_end_r) )
        
        # C. For each channel, Fit a line to this span of the recording
        for j in range( mCol ):
            if j != tC:
                tr   = X[ i_bgn_x:i_end_x, tC ].flatten()   #.reshape((1, -1))
                cr   = X[ i_bgn_x:i_end_x, j  ].flatten()   #.reshape((1, -1))
                
                
#                 [m, b] = np.polyfit( tr, cr, deg = 1 )
                [m, b] = slope_intercept_2d(  ( tr[0], cr[0] ), ( tr[-1], cr[-1] )  )


                # a. For each timestep on the returned side, predict the value of the channel at that time
                for i in range( i_bgn_r+1, intmax( i_end_r+1, rRow ) ):
                    # Populate returned matrix with interpolated element
    #                 R[i,j] = np.clip(  modl.predict( np.array( i*dt ).reshape((-1, 1)) ),  -1.0,  +1.0  )
                    x      = i*dt # np.array( i*dt ).reshape((-1, 1))
                    R[i,j] = np.clip(  m*x+b,  -1.0,  +1.0  )
    
        # D. Update the beginning of each segment
        i_bgn_x = i_end_x
        i_bgn_r = i_end_r
        
        # E. Increment the endpoints of the two sections
        i_end_x += 1
        i_end_x = intmin( i_end_x, xRow-1 ) 
        i_end_r += 1
        i_end_r = intmin( i_end_r, rRow-1 ) 

        # F. No matter what, the end of the recording equals the end of the returned series
        R[-1,: ] = X[-1,:]
    
    # N. Return
    return R


def interpolate_directory_and_save( inputDir, outptDir, dt, timeCol = 0 ):
    """ Load data from CSV directory, Noramlize it, then save it to NPY directory """
    
    # 0. Fetch all file names
    inpNames = os.listdir( inputDir )
    
    # 1. For every name in the input directory, build paths and interpolate to a standard uniform `dt`
    for inpNam in inpNames:
        print( "# Interpolate" , inpNam, "with dt =" , dt, end = "..." )
        # A. Build paths
        parts   = inpNam.split('.')
        outNam  = str(parts[:-1]).replace( "_RAW", "_INTR" ) + '.' + str(parts[-1])
        inpPath = os.path.join( inputDir, inpNam )
        outPath = os.path.join( outptDir, outNam )
        # B. Load file
        inpSeries = npy_to_array( inpPath )
        # C. Interpolate
        outSeries = interpolate_series( inpSeries, dt, timeCol )
        # D. Save
        np.save( outPath, outSeries, allow_pickle = 0 )
        print( "Done!" )



########## Feature Extraction #####################################################################


def statistical_moments( arr1D_np ):
    """ Get the mean, variance, skewness, and kurtosis """
    return {
        'mean' :     np.mean( arr1D_np ) , 
        'variance' : np.var( arr1D_np ) , 
        'skewness' : scipy.stats.skew( arr1D_np ) , 
        'kurtosis' : scipy.stats.kurtosis( arr1D_np ) ,
    }



########## Record Retrieval & Filtering ###########################################################


def rolling_average( data_np, winSize = 5, ignoreFirstCols = 1 ):
    """ Return a version of the data with a rolling average of every column after the `ignoreFirstCols-1` column """  
    # NOTE: Returned array will have the same shape as the input array, see below
    
    # 0. Get shape info
    ( Nrows , Mcols ) = data_np.shape
    bgnC = int( ignoreFirstCols )
    endC = Mcols
    
    # 1. Pre-allocate array && init
    rtnArr = np.zeros( data_np.shape )
    rtnArr[ 0, : ] = data_np[ 0, : ]
    
    # 2. Compute rolling averages
    for i in range( 1, Nrows ):
        # 3. If there is not a full window, use what you got
        if i < (winSize-1):
            bgnR = 0
        else:
            bgnR = i-winSize+1
        endR = i+1
        
        # 4. Average for this row
        rtnArr[ i, bgnC:endC ] = np.mean( data_np[ bgnR:endR, bgnC:endC ], 0 )
        if ignoreFirstCols:
            rtnArr[ i, 0 ] = data_np[ i, 0 ]
        
        
    # N. Return
    return rtnArr


def average_dataset_and_save( srcNPY, dstNPY, winSize, dtype = 'float32', ignoreFirstCols = 1 ):
    """ Load data from CSV, Noramlize it, then save it to NPY """
    
    # 1. Load the data
    data_NP = np.array( np.load( srcNPY ) ).astype( dtype = dtype )
    
    # 2. Smooth data
    smooth_NP = rolling_average( data_NP, winSize = winSize, ignoreFirstCols = ignoreFirstCols )
    
    # 3. Save data
    np.save( dstNPY, smooth_NP, allow_pickle = 0 )

    
# FIXME: USE THE NIST METHOD TO TUNE: https://www.itl.nist.gov/div898/handbook/pmc/section4/pmc431.htm

def Holt_Winters_double_exponential_window( data_np, alpha_dataFactor, beta_slopeFactor, ignoreFirstCols = 1 ):
    """ Apply the a version of the exponential window that has the capability to "follow trends" """
    # https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
    # # https://www.nist.gov/nist-research-library/reference-format-nist-publications
    # NOTE: Returned array will have the same shape as the input array, see below
    
    # 0. Get shape info
    ( Nrows , Mcols ) = data_np.shape
    bgnC = int( ignoreFirstCols )
    endC = Mcols
    
    # 1. Pre-allocate array
    rtnArr = np.zeros( data_np.shape )
    
    # 2. Init
    A   = alpha_dataFactor
    B   = beta_slopeFactor
    s_t = data_np[ 0, bgnC:endC ]
    b_t = data_np[ 1, bgnC:endC ] - data_np[ 0, bgnC:endC ]
    
    rtnArr[ 0, : ] = data_np[ 0, : ]
    
     # 2. Compute rolling exponential
    for i in range( 1, Nrows ):
        
        # A. Update the t-1 params
        s_tm1 = s_t
        b_tm1 = b_t
        
        # B. Compute new params
        s_t = A*data_np[ i, bgnC:endC ] + (1-A)*(s_tm1 + b_tm1) # Smoothed data
        b_t = B*(s_t - s_tm1) + (1-B)*b_tm1 # ------------------- Smoothed slope
        
        # C. Store the smoothed datapoint
        rtnArr[ i, bgnC:endC ] = s_t
        if ignoreFirstCols:
            rtnArr[ i, 0 ] = data_np[ i, 0 ]
        
    # N. Return
    return rtnArr


def Holt_Winters_dbbl_exp_ts( data_np, alpha_dataFactor, beta_slopeFactor, timeCol = 0 ):
    """ Timestep-aware double exponential window """
    # https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
    # https://www.itl.nist.gov/div898/handbook/pmc/section4/pmc434.htm
    # https://www.nist.gov/nist-research-library/reference-format-nist-publications
    # NOTE: Returned array will have the same shape as the input array, see below
    tC = timeCol
    dC = tC+1
    ( dRow, dCol ) = data_np.shape
    xRow = dRow
    xCol = dCol - dC
    
    # 0. Split into timestamps and data
    t = data_np[ : , tC  ].flatten()
    X = data_np[ : , dC: ]
    # 1. Allocate and pre-populate the return series
    R = np.zeros( ( dRow, dCol ) )
    R[:,tC] = t
    R[0,:]  = data_np[0,:]
    
    
    # 2. Init
    A   = alpha_dataFactor
    B   = beta_slopeFactor
    s_t = X[0,:]
    dt  = t[1] - t[0]
    b_t = ( X[1,:] - X[0,:] ) / dt
    t_i = t[0]
    
     # 2. Compute rolling exponential
    for i in range( 1, dRow ):
        
        # A. Update the t-1 params
        t_im1 = t_i
        s_tm1 = s_t
        b_tm1 = b_t
        dtm1  = dt
        
        # B. Compute new params
        t_i = t[i]
        dt  = t_i - t_im1
        # Smoothed data
        fctr = dt/dtm1 if (dt < dtm1) else dtm1/dt
        s_t  = A*X[i,:] + (1-A)*(s_tm1 + b_tm1*fctr) 
        # Smoothed slope
        m_t = (s_t - s_tm1)/dt
        b_t = B*m_t*fctr + (1-B)*b_tm1 
        
        # C. Store the smoothed datapoint
        R[ i, dC: ] = s_t
        
    # N. Return
    return R
        
        
def exp_window_dataset_and_save( srcNPY, dstNPY, alpha_dataFactor, beta_slopeFactor, dtype = 'float32', ignoreFirstCols = 1 ):
    """ Load data from CSV, Noramlize it, then save it to NPY """
    
    # 1. Load the data
    data_NP = np.array( np.load( srcNPY ) ).astype( dtype = dtype )
    
    # 2. Smooth data
    smooth_NP = Holt_Winters_double_exponential_window( data_NP, 
                                                        alpha_dataFactor, beta_slopeFactor, 
                                                        ignoreFirstCols = ignoreFirstCols )
    
    # 3. Save data
    np.save( dstNPY, smooth_NP, allow_pickle = 0 )



########## Plotting #######################################################################


def plot_FT( data_np ):
    """ Interpret `data_np` as force-torquw """
    
    x  = data_np[:,0]
    fx = data_np[:,1]
    fy = data_np[:,2]
    fz = data_np[:,3]
    tx = data_np[:,4]
    ty = data_np[:,5]
    tz = data_np[:,6]
    
    plt.plot( x , fx , label='Fx' )
    plt.plot( x , fy , label='Fy' )
    plt.plot( x , fz , label='Fz' )
    plt.xlabel( 'Time in millisecs' )
    plt.ylabel( 'Forces' )
    plt.title( 'Forces vs time ' )
    plt.legend()
    plt.show()

    plt.plot( x , tx , label='Tx' )
    plt.plot( x , ty , label='Ty' )
    plt.plot( x , tz , label='Tz' )
    plt.xlabel( 'Time in millisecs' )
    plt.ylabel( 'Torques' )
    plt.title( 'Torques vs time ' )
    plt.legend()
    plt.show()
    
def plot_CSV( csvPath ):
    plot_FT(  load_CSV_for_NN( csvPath , elemParser = float )  )
    
def box_compare( all_scores , names , graphTitle , savePath , showG = 1 , 
                 yBottom = 0.60 , yTop = 1.00 ):
    """ Make a box plot to compare models """
    plt.figure( figsize = ( 7 , 5 ) )
    plt.boxplot( all_scores , labels = names )
    plt.xticks( fontsize = 14 )
    plt.yticks( fontsize = 14 )
    plt.xlabel( 'Model' , fontsize = 14 )
    plt.ylabel( 'Accuracy' , fontsize = 14 )
    plt.title( graphTitle , fontsize = 14 , fontweight = 'bold' )
    plt.ylim( bottom = yBottom , top = yTop )
    plt.savefig( savePath , dpi = 300 , quality = 95 )
    if showG:
        plt.show()
        
def plot_series_pretty( series_np , timeCol = -1 , labels = [] , linWght = 2 ,
                        title = "Plot" , xLbl = "X Axis" , yLbl = "Y Axis" , size = [ 12.0 , 7.0 ] ):
    """ Plot `series_np` where each column is one time-series """
    
    plt.figure( figsize = size )
    
    Ncols = series_np.shape[1]
    
    if timeCol >= 0:
        T = series_np[ : , timeCol ]
        # 1. For each of the series
        i = 1
        for srsDex in range( Ncols ):
            if srsDex != timeCol:
                
                if labels:
                    lbl = labels[ srsDex ]
                else:
                    lbl = str( i )
                    i += 1
                
                plt.plot( series_np[ : , timeCol ] , # Independent X
                          series_np[ : , srsDex  ] , # Dependent   Y
                          label = lbl , linewidth = linWght )  # FIXME , LINE WEIGHT, START HERE
            
    else:
        # 1. For each of the series
        for srsDex in range( Ncols ):
            if labels:
                lbl = labels[ srsDex ]
            else:
                lbl = str( srsDex+1 )
            plt.plot( series_np[ : , srsDex ] , label = lbl , linewidth = linWght )  # FIXME , LINE WEIGHT, START HERE
        
    
    plt.xlabel( xLbl )
    plt.ylabel( yLbl )
    plt.title( title )
    plt.legend( loc = 'best' )
    plt.show()## 