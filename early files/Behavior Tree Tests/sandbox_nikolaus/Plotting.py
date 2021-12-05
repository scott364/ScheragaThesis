#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Template Version: 2016-07-08

##### MIT LICENSE BEGIN ##############################################################################
##                                                                                                  ##
##    Copyright 2018 James Watson, University of Colorado Boulder                                   ##
##                                                                                                  ##
##    Permission is hereby granted, free of charge, to any person obtaining a copy of this          ##
##    source file and associated documentation (the "File"), to deal in the File                    ##
##    without restriction, including without limitation the rights to use, copy, modify, merge,     ##
##    publish, distribute, sublicense, and/or sell copies of the File, and to permit persons        ##
##    to whom the File is furnished to do so, subject to the following conditions:                  ##
##                                                                                                  ##
##    The above copyright notice and this permission notice shall be included                       ##
##    in all copies or substantial portions of the File.                                            ##
##                                                                                                  ##
##    THE FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,               ##
##    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                               ##
##    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.                                         ##
##    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,                   ##
##    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,              ##
##    ARISING FROM, OUT OF OR IN CONNECTION WITH THE FILE                                           ##
##    OR THE USE OR OTHER DEALINGS IN THE FILE.                                                     ##
##                                                                                                  ##
##### MIT LICENSE END ################################################################################

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

"""
Plotting.py
James Watson, 2016 July
Useful functions for plotting, mostly in matplotlib
"""

# ~ Special Libraries ~
import numpy as np
import matplotlib
# matplotlib.use('Qt4Agg') # Not even sure what this is for
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

COMMONCOLORS = [
    'b' , # blue.
    'g' , # green.
    'r' , # red.
    'c' , # cyan.
    'm' , # magenta.
    'k'   # black.
]

def rolling_rainbow( index ):
    return COMMONCOLORS[ index % len( COMMONCOLORS ) ]

def split_to_components( vecList ):
    """ Separate a list of R3 vectors into three lists of components """ # because matplotlib likes it that way
    plotXs = []
    plotYs = []
    plotZs = []
    for vec in vecList:
#        print vec
        plotXs.append( vec[0] )
        plotYs.append( vec[1] )
        plotZs.append( vec[2] )
    return plotXs, plotYs, plotZs
    
def split_to_comp_cycle( vecList ):
    """ Separate a list of R3 vectors into three lists of components, in which the last item is a duplicate of the first """
    plotXs, plotYs, plotZs = split_to_components( vecList )
    plotXs.append( plotXs[0] )
    plotYs.append( plotYs[0] )
    plotZs.append( plotZs[0] )
    return plotXs, plotYs, plotZs
    
def split_to_2D( vecList ): 
    """ Separate a list of R2 vectors into two lists of components """ # because matplotlib likes it that way
    # NOTE: This will technically work for vectors of any dimensionaly 2 and above, indices > 1 will be ignored
    plotXs = []
    plotYs = []
    for vec in vecList:
        plotXs.append( vec[0] )
        plotYs.append( vec[1] )
    return plotXs, plotYs
    
def split_2d_on_XY_Zeq0( vecList ): 
    """ Separate a list of R2 vectors into three lists of components on the X-Y plane, Z = 0 """ 
    plotXs = []
    plotYs = []
    plotZs = []
    for vec in vecList:
        plotXs.append( vec[0] )
        plotYs.append( vec[1] )
        plotZs.append(      0 )
    return plotXs, plotYs, plotZs    
    
def plot_axes_3D_mpl(plotAX, scale = 1): 
    """ Display the coordinate axes in standard XYZ-RGB on a matplotlib 3D projection, each vector 'scale' in length """
    # NOTE: This function assumes that 'axes' has already been set up as a 3D projection subplot
    # URL, Show Origin: http://stackoverflow.com/a/11541628/893511
    # ax.plot( Xs    , Ys    , Zs    , c=COLORNAME )
    plotAX.plot( [0,scale] , [0,0    ] , [0,0    ] , c='r')
    plotAX.plot( [0,0    ] , [0,scale] , [0,0    ] , c='g')
    plotAX.plot( [0,0    ] , [0,0    ] , [0,scale] , c='b')
    
def plot_bases_3D_mpl( plotAX , origin , bX , bY , bZ , scale , labelNum = None , labelStr = None ): 
    """ Display the supplied axes in standard XYZ-RGB on a matplotlib 3D projection, each vector 'scale' in length """
    # NOTE: This function assumes that 'axes' has already been set up as a 3D projection subplot
    # NOTE: This function does not perform any checks whatsoever on the orthogonality or length of bX, bY, bZ
    # URL, Show Origin: http://stackoverflow.com/a/11541628/893511
    # ax.plot( Xs    , Ys    , Zs    , c=COLORNAME )
    xVec = np.add( np.multiply( bX , scale ) , origin)
    yVec = np.add( np.multiply( bY , scale ) , origin)
    zVec = np.add( np.multiply( bZ , scale ) , origin)
    Xs, Ys, Zs = split_to_components( [ origin , xVec ] )
    plotAX.plot( Xs, Ys, Zs , c='r')
    Xs, Ys, Zs = split_to_components( [ origin , yVec ] )
    plotAX.plot( Xs, Ys, Zs , c='g')
    Xs, Ys, Zs = split_to_components( [ origin , zVec ] )
    plotAX.plot( Xs, Ys, Zs , c='b')
    if  ( labelNum != None )  or  ( labelStr != None ):
        if labelNum != None:
            subScript = str(labelNum)
        else:
            subScript = str(labelStr)
        
        labelBeyondFactor = 1.1
        
        xLoc = np.add( np.multiply( bX , scale * labelBeyondFactor ) , origin)
        yLoc = np.add( np.multiply( bY , scale * labelBeyondFactor ) , origin)
        zLoc = np.add( np.multiply( bZ , scale * labelBeyondFactor ) , origin)
        
        plotAX.text( xLoc[0] , xLoc[1] , xLoc[2] , "x_" + subScript , tuple( bX ) )
        plotAX.text( yLoc[0] , yLoc[1] , yLoc[2] , "y_" + subScript , tuple( bY ) )
        plotAX.text( zLoc[0] , zLoc[1] , zLoc[2] , "z_" + subScript , tuple( bZ ) )

def apply_homog( homogMat , vec3 ):
    """ Apply a homogeneous transformation to a 3D vector """
#     print( "homogMat:\n" , homogMat , "vec3:" , vec3 )
    rtnVec = ( np.dot( homogMat , [ vec3[0] , vec3[1] , vec3[2] , 0.0 ] ) )
    #rtnVec = ( np.dot( homogMat , np.array( [ [vec3[0]] , [vec3[1]] , [vec3[2]] , [1.0] ] ) ) )
#     print( "rtnVec:" , rtnVec )
    return rtnVec[:3]
    
def get_basis_vectors_for_xform( xform ):
    """ Return the basis vector for the transformation """
    xBasis = apply_homog( xform , [1,0,0] )
    yBasis = apply_homog( xform , [0,1,0] )
    zBasis = apply_homog( xform , [0,0,1] )
    return xBasis , yBasis , zBasis    
 
def get_position( xform ):
    """ Set the translation portion of the `xform` to `pos` """
    return [ xform[0,3] , xform[1,3] , xform[2,3] ]
    
def plot_pose_axes_mpl( plotAX , homogPose , scale , labelNum = None , labelStr = None ):
    """ Represent a Vector.Pose as 3D bases in a plot """
    xBasis , yBasis , zBasis = get_basis_vectors_for_xform( homogPose )
#     print( xBasis , yBasis , zBasis )
    plot_bases_3D_mpl(plotAX, 
                      get_position( homogPose ) , 
                      xBasis , yBasis , zBasis, 
                      scale,
                      labelNum,
                      labelStr)

# == Figure Create / Destroy ==

def fig_num(): fig_num.num += 1 ; return fig_num.num # Functor to increment and return figure number with each call
fig_num.num = 0
 
def fig_3d( dims_in = [ 6.4 , 4.8 ] ): # fix , ax = fig_3d()
    """ Create a new 3D figure and return handles to figure and axes """
    # USAGE: fig , ax = fig_3d()
    fig = plt.figure( num = fig_num() , figsize = dims_in )
    ax = fig.add_subplot( 111 , projection = '3d' )
    return fig , ax
    
def show_3d():
    """ Show all the 3D figures, should only be called once per program """
    #plt.axis('equal')
    plt.show()

def fig_2d():
    """ Create a new 2D figure (default) and return handles to figure and axes """
    fig = plt.figure( fig_num() )
    ax = fig.add_subplot( 111 )
    return fig , ax

def show_2d():
    """ Show all the 2D figures, should only be called once per program """
    plt.show()

def close_all_figs():
    """ Close all open figures """
    plt.close('all')

# __ End Figure __
    
def ax_view( ax , azimuth , elevation ):
    """ Set the camera view for the axes """
    ax.view_init( azimuth , elevation )
    
def plot_points_to_ax( ax , ptsList , size = 14 , color = 'blue' , mrkr = 'o' ):
    xs , ys , zs = split_to_components( ptsList )
    ax.scatter( xs , ys , zs , c = color , marker = mrkr , s = size )

def plot_points_only_list( ptsList , size = 14 , color = 'blue' , mrkr = 'o' , paintAxes = False ):
    """ Plot the uniqueified points already stored in a list """ # NOTE: This function assumes a points-only file exists!

    fig = plt.figure()
    ax  = fig.add_subplot( 111 , projection = '3d' )
    
    plot_points_to_ax( ax , ptsList , size , color , mrkr  )
    
    if paintAxes:
        plot_axes_3D_mpl( ax , scale = 0.05 )
    
    # plt.gca().set_aspect('equal')
    axes_equal( ax )
    plt.show()
    
def plot_chain_to_ax( ax , ptsList , makeCycle = False , color = 'blue' , width = 1 ):
    xs , ys , zs = split_to_components( ptsList )
    if makeCycle:
        for coordList in [ xs , ys , zs ]:
            coordList.append( coordList[0] )
    ax.plot( xs , ys , zs , c = color , linewidth = width )
    
def plot_chain( ptsList , makeCycle = False , color = 'blue' , paintAxes = False ):
    """ Plot the uniqueified points already stored in a list """ # NOTE: This function assumes a points-only file exists!
    fig = plt.figure()
    #ax = Axes3D(fig) 
    ax = fig.add_subplot( 111 , projection = '3d' )
    #pointsOnly = load_pkl_struct(PKLpath) # You must be aware of the structure in order to use it
    
    plot_chain_to_ax( ax , ptsList , makeCycle , color )
    
    if paintAxes:
        plot_axes_3D_mpl( ax , scale = 0.05 )
    
    plt.gca().set_aspect( 'equal' )
    plt.show() 
    
def plot2D_chain_to_ax( ax , ptsList , makeCycle = False , color = 'blue' , width = 1 ):
    xs , ys = split_to_2D( ptsList )
    if makeCycle:
        for coordList in [ xs , ys ]:
            coordList.append( coordList[0] )
    ax.plot( xs , ys , c = color , linewidth = width )
    
def plot_VF_to_ax( ax , V , F , color = 'blue' , width = 1 ):
    """ Plot wireframe trimesh to axes """
    for i , f_i in enumerate( F ):
        triPts = []
        for vDex in f_i:
            triPts.append( V[vDex][:] )
        plot_chain_to_ax( ax , triPts , True , color , width )

def axes_equal( ax ):
    """ Use the pre-stretch axes limits to stretch the axes to equal scales """
    xDataRange = ax.get_xlim();  xCen = ( xDataRange[1] + xDataRange[0] ) / 2.0;  lngth = abs( xDataRange[1] - xDataRange[0] )
    yDataRange = ax.get_ylim();  yCen = ( yDataRange[1] + yDataRange[0] ) / 2.0;  width = abs( yDataRange[1] - yDataRange[0] )
    zDataRange = ax.get_zlim();  zCen = ( zDataRange[1] + zDataRange[0] ) / 2.0;  depth = abs( zDataRange[1] - zDataRange[0] )
    span = max( [ lngth , width , depth ] )
    
    xLims = [ xCen - span/2.0 , xCen + span/2.0 ]
    yLims = [ yCen - span/2.0 , yCen + span/2.0 ]
    zLims = [ zCen - span/2.0 , zCen + span/2.0 ]
    
    corners = []
    
    for x in xLims:
        for y in yLims:
            for z in zLims:
                corners.append( [ x , y , z ] )
                
    plot_points_to_ax( ax , corners , size = 0.01 , color = 'white' , mrkr = '.' )


# == IPython / Jupyter ==

def ipy_set_plots_interactive( ):
    # ~ MPL Mode ~
    get_ipython().magic( 'matplotlib widget' ) # %matplotlib widget
    
#     if allowRotate:
#         get_ipython().magic( 'matplotlib notebook' ) # Interactive plots
#     else:
#         get_ipython().magic( 'matplotlib inline' ) # Static plots

# __ End Jupyter __

if __name__ == "__main__":
    fg , ax = fig_3d()
    plot_axes_3D_mpl( ax , scale = 2 )
    pose = np.array(  
        [ [ 1, 0, 0, 1 ] ,
          [ 0, 1, 0, 1 ] ,
          [ 0, 0, 1, 1 ] ,
          [ 0, 0, 0, 1 ] ]
    )
    plot_pose_axes_mpl( ax , pose , 2 , labelNum = 1 )
    show_3d()
        
# === SPARE PARTS ==========================================================================================================================
        

            
    
#    ax.set_xlim(  );
#    ax.set_ylim(  );
#    ax.set_zlim(  );
#    ax.set_xbound( xCen - span/2.0 , xCen + span/2.0 );
#    ax.set_ybound( yCen - span/2.0 , yCen + span/2.0 );
#    ax.set_zbound( zCen - span/2.0 , zCen + span/2.0 );
    
#    ax.set_aspect( 'equal' , adjustable = 'box' )
        
# ___ END SPARE ____________________________________________________________________________________________________________________________
