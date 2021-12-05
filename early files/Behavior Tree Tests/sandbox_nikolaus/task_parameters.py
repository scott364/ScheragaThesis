import numpy as np
from math import sin , cos , radians
from utils import vec_unit

_TB_side      = 0.384 # Used multiple
_BIN_width    = 0.250
_BIN_length   = 0.315
_M8_TB_height = 0.008 # 0.010 # Used multiple
_M6_TB_height = 0.007 # Used multiple
_M4_TB_height = 0.004 # Used multiple

taskParams = {
    
    # ~~ Constant Parameters ~~
    'TB_width'          : _TB_side ,
    'TB_length'         : _TB_side ,
    'TB_deck-guess-Z'   : 0.030 ,
    'BIN_lip-height'    : 0.055 ,
    'M4PITCHm'          : 0.00070 ,
    'M6PITCHm'          : 0.00100 ,
    'M8PITCHm'          : 0.00125 ,
    'WS_width'          : 1.000 ,
    'WS_length'         : 0.600 ,
    'kit_view_distance' : 0.150 ,
    'CI_OFFSETS_0.010'  : [-0.0336, -0.0385] ,
    'CI_OFFSETS_0.040'  : [-0.0327, -0.0409] , 
    'CI_OFFSETS_0.070'  : [-0.0326, -0.0435] ,
    'CI_OFFSETS_0.100'  : [-0.0333, -0.0454] ,
    'CI_OFFSETS_SAVED'  : [-0.0355, -0.0451] ,
    
    # ~~ HACKS ~~
    'drill-offset'        : np.array( [ 0.008 , 0.002 ] ) ,
    'HAX_boardOffset'     : [ 0.008 , 0.002 , 0.000 ] ,
    'HAX_screwPickOffset' : [ 0.005 , 0.001 , 0.000 ] ,
    'HAX_kitPickOffset'   : [ 0.000 , -0.004 , 0.000 ] ,
    
    # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # TASKBOARD # 
    # ~~ NIST Taskboard (ENTER MINUTE 0 , TASK 1) ~~ # TB # TB # TB # TB # TB # TB # TB # TB # TB # TB # TB # TB # TB # TB 
    'TB_deck-height' : 0.028 ,
    #    +----------+
    #    |   +-->X  |
    #    |   v HAND |    Taskboard finder will assign 
    #    Y   Y      |    the taskboard axis nearest X_hand as X_board
    #    ^          |   
    #    +-->X--TASKBOARD
    #           ___     
    #          /   \    
    #          ROBOT
    #          \___/
    # ~ Single Items ~
    'gear-big_shaft_XYZ' :       np.array( [ 0.310 , 0.140 , 0.020 ] ) ,
    'gear-sml_shaft_XYZ' :       np.array( [ 0.340 , 0.140 , 0.020 ] ) ,
    
    'shaft8_hole_XYZ'    :       np.array( [ 0.235 , 0.030 , 0.000 ] ) , # HOLES ARE FLUSH
    'shaft16_hole_XYZ'   :       np.array( [ 0.255 , 0.160 , 0.000 ] ) ,
    
    'BNC-port_top_XYZ' :         np.array( [ 0.240 , 0.100 , 0.020 ] ) ,
    
    'KET4-end_XYZ' :             np.array( [ 0.300 , 0.090 , 0.020 ] ) ,
    'KET12-end_XYZ' :            np.array( [ 0.275 , 0.045 , 0.020 ] ) ,
    'KET12-long_dir' :           np.array( [ 0.000 , 1.000 , 0.000 ] ) ,
    
    'ethernet-jack_XYZ' :        np.array( [ 0.340 , 0.050 , 0.037 ] ) ,
    'ethernet-mount_screw-dir' : np.array( [ 0.000 , 1.000 , 0.000 ] ) , # Near to far
    'ethernet-mount_pin-dir' :   np.array( [ 0.000 , 1.000 , 0.000 ] ) , 
    
    'pulley-big_top_XYZ' :       np.array( [ 0.150 , 0.250 , 0.030 ] ) ,
    'pulley-sml_top_XYZ' :       np.array( [ 0.055 , 0.330 , 0.030 ] ) ,
    
    'USB-plug-cntr_XYZ' :        np.array( [ 0.350 , 0.225 , 0.040 ] ) ,
    'USB-plug_dir' :             np.array( [ 0.000 , 1.000 , 0.000 ] ) , 
    'Cable-route-post1_XYZ' :    np.array( [ 0.230 , 0.260 , 0.035 ] ) , 
    'Cable-route-anchor1_dir' :  np.array( [ 1.000 , 0.000 , 0.000 ] ) , 
    'Cable-route-post2_XYZ' :    np.array( [ 0.260 , 0.325 , 0.035 ] ) , 
    'Cable-route-anchor2_dir' :  np.array( [ 1.000 , 0.000 , 0.000 ] ) ,
    'USB-plug-long_dir' :        np.array( [ 0.000 , 1.000 , 0.000 ] ) ,
    'USB-anchor-cntr_XYZ' :      np.array( [ 0.225 , 0.370 , 0.013 ] ) , 
    # ~ M8 ~
    'M8_board-XYZ_1' : np.array( [ 0.040 , 0.140 , _M8_TB_height ] ) ,
    'M8_board-XYZ_2' : np.array( [ 0.065 , 0.115 , _M8_TB_height ] ) ,
    'M8_board-XYZ_3' : np.array( [ 0.090 , 0.040 , _M8_TB_height ] ) ,
    'M8_board-XYZ_4' : np.array( [ 0.115 , 0.065 , _M8_TB_height ] ) ,
    'M8_board-XYZ_5' : np.array( [ 0.140 , 0.090 , _M8_TB_height ] ) ,
    'M8_board-XYZ_6' : np.array( [ 0.165 , 0.165 , _M8_TB_height ] ) ,
    # ~ M6 ~
    'M6_board-XYZ_1' : np.array( [ 0.040 , 0.090 , _M6_TB_height ] ) ,
    'M6_board-XYZ_2' : np.array( [ 0.065 , 0.065 , _M6_TB_height ] ) ,
    'M6_board-XYZ_3' : np.array( [ 0.065 , 0.165 , _M6_TB_height ] ) ,
    'M6_board-XYZ_4' : np.array( [ 0.115 , 0.165 , _M6_TB_height ] ) ,
    'M6_board-XYZ_5' : np.array( [ 0.165 , 0.065 , _M6_TB_height ] ) ,
    'M6_board-XYZ_6' : np.array( [ 0.165 , 0.115 , _M6_TB_height ] ) ,
    # ~ M4 ~
    'M4_board-XYZ_1' : np.array( [ 0.040 , 0.040 , _M4_TB_height ] ) ,
    'M4_board-XYZ_2' : np.array( [ 0.090 , 0.090 , _M4_TB_height ] ) ,
    'M4_board-XYZ_3' : np.array( [ 0.090 , 0.140 , _M4_TB_height ] ) ,
    'M4_board-XYZ_4' : np.array( [ 0.115 , 0.115 , _M4_TB_height ] ) ,
    'M4_board-XYZ_5' : np.array( [ 0.140 , 0.040 , _M4_TB_height ] ) ,
    'M4_board-XYZ_6' : np.array( [ 0.140 , 0.140 , _M4_TB_height ] ) ,
    
    # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # 
    # ~~ NIST Kit (ENTER MINUTE 0 , TASK 2) ~~ # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT # KIT 
    'kit_dims' :            ( 0.250 , 0.250 ) , # Verify
    
    'belt-cntr_kit-XYZ'  :  np.array( [ 0.150 , 0.100 , 0.000 ] ) , # 0.004 belt thick
    
    'KET12_top_kit-XYZ' :   np.array( [ 0.150 , 0.220 , 0.050 ] ) ,
    'KET12-long_kit-DIR' :  np.array( vec_unit( [ 0.000 , 1.000 , 0.000 ] ) ) ,
    
    'gear-big_kit-XYZ' :    np.array( [ 0.085 , 0.185 , 0.020 ] ) ,
    'gear-sml_kit-XYZ' :    np.array( [ 0.050 , 0.115 , 0.020 ] ) ,
    
    'shaft8_kit-XYZ'   :    np.array( [ 0.225 , 0.050 , 0.050 ] ) ,
    'shaft16_kit-XYZ'  :    np.array( [ 0.200 , 0.175 , 0.050 ] ) ,
    
    'BNC-plug_kit-XYZ' :    np.array( [ 0.100 , 0.025 , 0.020 ] ) ,
    
    'KET4_cntr_kit-XYZ' :   np.array( [ 0.025 , 0.175 , 0.004 ] ) ,
    'KET4_kit-DIR' :        np.array( vec_unit( [ 0.000 , 1.000 , 0.000 ] ) ) ,
    
    'ether-cntr_kit-XYZ' :  np.array( [ 0.055 , 0.060 , 0.010 ] ) ,
    'either-plug_kit-DIR' : np.array( vec_unit( [ 0.000 , 1.000 , 0.000 ] ) ) ,
    
    
    
    # ~~ Features (Test in new lighting) ~~
    'KT_locator' : {
        'capture_process_list': [
            {'max_rad_px': 57, 'param_2': 29, 'min_dist_px': 104.0, 'param_1': 55, 'blur': 5, 'descriptor': 'center_circle',
                                  'search_radius_px': 500, 'min_rad_px': 47}
        ], 
        'view_distance': 0.07
    } ,
    'TB_locator' : {
        'view_distance': 0.20 ,
        'capture_process_list':
        [
            {
                'descriptor': 'downsample',
                'leaf_size': 0.005 ,
            },
#             {
#                  'descriptor': 'remove_plane',
#                  'plane_tol' : 0.003
#             },
            {
                'descriptor': 'dbscan',
                'min_samples': 15 , # 15
                'search_radius': 0.010 ,
            },
#             {
#                 'descriptor': 'sort_clouds_size',
#                 'large_to_small': True,
#             },
            {
                'descriptor': 'filter_by_size',
                'cluster_size': [7, 10000000],
                'max_clouds_returned': 100,
                'x_axis': [ _TB_side * 0.80 , _TB_side * 1.10 ],
                'y_axis': [ _TB_side * 0.80 , _TB_side * 1.10 ],
                'z_axis': [0, 0],
            },
        ]
    } ,
    'TB_locator_ASM' : {
        'view_distance': 0.10 ,
        'capture_process_list':
        [
            {
                'descriptor': 'downsample',
                'leaf_size': 0.004 ,
            },
#             {
#                  'descriptor': 'remove_plane',
#                  'plane_tol' : 0.003
#             },
            {
                'descriptor': 'dbscan',
                'min_samples':  10 , # 15
                'search_radius': 0.010 ,
            },
#             {
#                 'descriptor': 'sort_clouds_size',
#                 'large_to_small': True,
#             },
            {
                'descriptor': 'filter_by_size',
                'cluster_size': [7, 10000000],
                'max_clouds_returned': 100,
                'x_axis': [ _TB_side * 0.80 , 0.50 ],
                'y_axis': [ _TB_side * 0.80 , 0.50 ],
                'z_axis': [0, 0],
            },
            { 
                'descriptor':  'sort_clouds_height', 
                'high_to_low': 0
            },
        ]
    } ,
    'TB_objects' : {
        'cable-post_feature': {'view_distance': 0.07, 'capture_process_list': [{'param_2': 26, 'descriptor': 'center_circle', 'search_radius_px': 300, 'max_rad_px': 66, 'min_rad_px': 56, 'param_1': 14, 'blur': 7, 'min_dist_px': 122.0}]} ,
        'shaft16-hole_feature' : { 
            'view_distance': 0.07 , 
            'capture_process_list': [
                {'blur': 7, 'param_2': 39, 'min_dist_px': 82.0, 'param_1': 57, 'search_radius_px': 500, 'max_rad_px': 46, 'min_rad_px': 36, 
                 'descriptor': 'center_circle'}
            ]
        } ,
        'shaft8-hole_feature'  : {
            'view_distance': 0.03 , 
            'capture_process_list': [
                {'blur': 7, 'param_2': 47, 'min_dist_px': 52.0, 'param_1': 47, 'search_radius_px': 500, 'max_rad_px': 31, 'min_rad_px': 21, 
                 'descriptor': 'center_circle'}
            ]
        } ,
        'gear-shaft_feature' : {'capture_process_list': [{'min_dist_px': 28.0, 'param_1': 21, 'param_2': 18, 'max_rad_px': 19, 'search_radius_px': 150, 'blur': 3, 'descriptor': 'center_circle', 'min_rad_px': 9}], 'view_distance': 0.015} ,
        'BNC-port_feature' : 
{'capture_process_list': [{'search_radius_px': 175 , 'min_dist_px': 68.0, 'param_1': 10, 'max_rad_px': 39, 'descriptor': 'center_circle', 'min_rad_px': 29, 'blur': 11, 'param_2': 15}], 'view_distance': 0.02} ,
        'pulley-big_feature' : {
            'view_distance': 0.07, 
            'capture_process_list': [
                {'min_rad_px': 123, 'max_rad_px': 142, 'param_2': 44, 'search_radius_px': 500, 'min_dist_px': 266.0, 'blur': 5, 'param_1': 17,
                 'descriptor': 'center_circle'}
            ]
        },
        'pulley-sml_feature' : {
            'view_distance': 0.07, 
            'capture_process_list': [
                {'min_rad_px': 84, 'max_rad_px': 97, 'param_2': 31, 'search_radius_px': 500, 'min_dist_px': 182.0, 'blur': 5, 'param_1': 48, 
                 'descriptor': 'center_circle'}
            ]
        },
        'ethernet-mount_feature' : {
            'view_distance': 0.08, 
            'capture_process_list': [
                {'min_dist_px': 58.0, 'param_2': 26, 'search_radius_px': 200, 'param_1': 60, 'max_rad_px': 34, 'descriptor': 'center_circle', 
                 'blur': 0, 'min_rad_px': 24}
            ]
        } ,
        
        'M8-hole_feature' : {'view_distance': 0.02, 'capture_process_list': [{'blur': 3, 'param_1': 39, 'search_radius_px': 300, 'max_rad_px': 29, 'min_rad_px': 19, 'descriptor': 'center_circle', 'param_2': 42, 'min_dist_px': 48.0}]} ,
        
        'M6-hole_feature' : {'capture_process_list': [{'param_2': 31, 'min_rad_px': 15, 'blur': 0, 'descriptor': 'center_circle', 'max_rad_px': 25, 'search_radius_px': 250, 'param_1': 45, 'min_dist_px': 40.0}], 'view_distance': 0.0} ,
        
        'M4-hole_feature' : {'capture_process_list': [{'param_2': 23, 'min_rad_px': 7, 'blur': 0, 'descriptor': 'center_circle', 'search_radius_px': 250, 'param_1': 49, 'max_rad_px': 17, 'min_dist_px': 24.0}], 'view_distance': 0.0} ,
        
        'ethernet-mount_screw-sep' : 0.0283 , 
    } ,
    'kit_objects' : {
        'gear-big_feature' : {
            'view_distance': 0.07, 
            'capture_process_list': [
                {'param_2': 22, 'min_dist_px': 142.0, 'param_1': 15, 'search_radius_px': 250, 'max_rad_px': 76, 'blur': 5, 'min_rad_px': 66, 
                 'descriptor': 'center_circle'}
            ]
        } ,
        
        'gear-big_cloud': {'capture_process_list': [{'descriptor': 'remove_plane',
            'plane_tol': 0.003},
           {'descriptor': 'dbscan', 'min_samples': 10, 'search_radius': 0.01},
           {'cluster_size': [40, 10000000],
            'descriptor': 'filter_by_size',
            'max_clouds_returned': 100,
            'x_axis': [0.035, 0.045],
            'y_axis': [0.035, 0.045],
            'z_axis': [0, 0]}],
          'view_distance': 0.15},
        
        'gear-sml_feature' : {
            'view_distance': 0.07, 
            'capture_process_list': [
                {'param_2': 22, 'min_dist_px': 76.0, 'param_1': 13, 'search_radius_px': 150, 'max_rad_px': 43, 'blur': 9, 'min_rad_px': 33, 
                 'descriptor': 'center_circle'}
            ]
        } ,
        
        'gear-sml_feature_INV' : {'view_distance': 0.02, 'capture_process_list': [{'blur': 9, 'min_rad_px': 29, 'descriptor': 'center_circle', 'search_radius_px': 500, 'min_dist_px': 68.0, 'param_2': 39, 'max_rad_px': 39, 'param_1': 59}]},
        
        'shaft16_feature' : {
            'view_distance': 0.07, 
            'capture_process_list': [
                {'blur': 3, 'param_2': 38, 'min_dist_px': 82.0, 'param_1': 45, 'search_radius_px': 200, 'max_rad_px': 46, 'min_rad_px': 36, 
                 'descriptor': 'center_circle'}
            ]
        } , 
        'shaft8_feature'  : {'capture_process_list': [{'max_rad_px': 33, 'search_radius_px': 200, 'descriptor': 'center_circle', 'blur': 3, 'param_2': 23, 'min_dist_px': 56.0, 'min_rad_px': 23, 'param_1': 10}], 'view_distance': 0.015} , 
        'BNC-plug_feature' : {
            'view_distance': 0.04, 
            'capture_process_list': [
                {'blur': 9, 'param_1': 46, 'max_rad_px': 47, 'search_radius_px': 250, 'min_rad_px': 37, 'min_dist_px': 84.0, 
                 'descriptor': 'center_circle', 'param_2': 26}
            ]
        } ,
        
        'M8_Feature' : {'view_distance': 0.015, 'capture_process_list': [{'min_dist_px': 94.0, 'blur': 5, 'descriptor': 'center_circle', 'search_radius_px': 250, 'max_rad_px': 52, 'param_1': 29, 'min_rad_px': 42, 'param_2': 55}]} ,
        
        'M6_Feature' : {'capture_process_list': [{'max_rad_px': 41, 'param_2': 55, 'search_radius_px': 200, 'descriptor': 'center_circle', 'param_1': 37, 'min_dist_px': 72.0, 'blur': 7, 'min_rad_px': 31}], 'view_distance': 0.015} ,
        
        'M4_Feature' : {'capture_process_list': [{'param_1': 23, 'blur': 7, 'max_rad_px': 30, 'search_radius_px': 250, 'descriptor': 'center_circle', 'min_dist_px': 50.0, 'min_rad_px': 20, 'param_2': 29}], 'view_distance': 0.015} , 
        
        
        'ether_feature' : {
            'view_distance': 0.150 ,
            'capture_process_list': [
                {
                    'descriptor': 'downsample',
                    'leaf_size': 0.003,
                },
                 {'descriptor': 'remove_plane', 'plane_tol': 0.003},
                {
                    'descriptor': 'dbscan',
                    'min_samples': 10,
                    'search_radius': 0.010,
                },
                {
                    'descriptor': 'filter_by_size',
                    'cluster_size': [7, 10000000],
                    'max_clouds_returned': 100,
                    'x_axis': [0.016, 0.140],
                    'y_axis': [0.008, 0.020],
                    'z_axis': [0, 0],
                },
                {
                    'descriptor': 'sort_clouds_distance',
                    'close_to_far': True,
                },
                {'descriptor': 'find_grasp', 'rotate_z': 1},
            ] ,
        },
        # View distance below 0.050 doesn't work?
        'KET12_feature_TB' : {
            'view_distance': 0.070, 'capture_process_list': [
            {'descriptor': 'downsample', 'leaf_size': 0.0015}, 
            {'descriptor': 'remove_plane', 'plane_tol': 0.01}, 
            {'search_radius': 0.005, 'descriptor': 'dbscan', 'min_samples': 3}, 
            {'descriptor': 'filter_by_size', 'cluster_size': [5, 10000000], 'z_axis': [0, 0], 'x_axis': [0.004, 0.015], 'max_clouds_returned': 100, 'y_axis': [0.004, 0.018]}, 
            {'descriptor': 'sort_clouds_distance',
                    'close_to_far': True,
                }, {'descriptor': 'find_grasp', 'rotate_z': 1}]
        },
        'KET12_feature_ASM' : {
            'view_distance': 0.080, 'capture_process_list': [
            {'descriptor': 'downsample', 'leaf_size': 0.0015}, 
            {'descriptor': 'remove_plane', 'plane_tol': 0.01}, 
            {'search_radius': 0.005, 'descriptor': 'dbscan', 'min_samples': 3}, 
            {'descriptor': 'filter_by_size', 'cluster_size': [2, 10000000], 'z_axis': [0, 0], 'x_axis': [0.004, 0.015], 'max_clouds_returned': 100, 'y_axis': [0.004, 0.018]}, 
            {'descriptor': 'sort_clouds_distance',
                    'close_to_far': True,
                }, {'descriptor': 'find_grasp', 'rotate_z': 1}]
        },
        
#         'KET12_feature' : {'view_distance': 0.070, 'capture_process_list': [{'descriptor': 'downsample', 'leaf_size': 0.0015}, {'descriptor': 'remove_plane', 'plane_tol': 0.01}, {'search_radius': 0.01, 'descriptor': 'dbscan', 'min_samples': 3}, {'descriptor': 'filter_by_size', 'cluster_size': [5, 10000000], 'z_axis': [0, 0], 'x_axis': [0, 0.015], 'max_clouds_returned': 100, 'y_axis': [0, 0.015]}, {'descriptor': 'sort_clouds_height', 'high_to_low': 1}, {'descriptor': 'find_grasp', 'rotate_z': 1}]},
        
        'KET4-end_feature' : {
            'view_distance': 0.030 ,
            'capture_process_list':
            [
                {
                    'descriptor': 'downsample',
                    'leaf_size': 0.0005,
                },
                {
                    'descriptor': 'dbscan',
                    'min_samples': 5,
                    'search_radius': 0.007,
                },
                {
                    'descriptor': 'filter_by_size',
                    'cluster_size': [7, 10000000],
                    'max_clouds_returned': 100,
                    'x_axis': [0.002, 0.007],
                    'y_axis': [0.002, 0.007],
                    'z_axis': [0, 0],
                },
                {
                    'descriptor': 'sort_clouds_distance',
                    'close_to_far': True,
                },
                {
                    'descriptor': 'find_pose',
                    'rotate_z': True # Rotates EVERYTHING
                },
            ] ,
        },
        "belt-pict_feature" : {'view_distance': 0.1, 'capture_process_list': [{'descriptor': 'center_circle', 'min_dist_px': 452.0, 'blur': 0, 'search_radius_px': 500, 'max_rad_px': 241, 'min_rad_px': 210, 'param_2': 54, 'param_1': 56}]} ,
    } ,
    'bin_locator' : { 
        'view_distance': 0.15 ,
        'capture_process_list':
        [
            {
                'descriptor': 'downsample',
                'leaf_size': 0.002,
            },
#             {
#                  'descriptor': 'remove_plane',
#                  'plane_tol' : 0.003
#             },
            {
                'descriptor': 'dbscan',
                'min_samples': 22 ,
                'search_radius': 0.005 ,
            },
#             {
#                 'descriptor': 'sort_clouds_size',
#                 'large_to_small': True,
#             },
            {
                'descriptor': 'filter_by_size',
                'cluster_size': [7, 10000000],
                'max_clouds_returned': 100,
                'x_axis': [ _BIN_width  * 0.80 , _BIN_width  * 1.05 ],
                'y_axis': [ _BIN_length * 0.80 , _BIN_length * 1.05 ],
                'z_axis': [0, 0],
            },
            {
                'descriptor': 'find_pose',
                'rotate_z': True # Rotates EVERYTHING
            },
        ]
    } ,
    'scraper_locator' : {
        'view_distance': 0.20 ,
        'capture_process_list':
        [
            {
                'descriptor': 'downsample',
                'leaf_size': 0.005,
            },
            {
                'descriptor': 'dbscan',
                'min_samples': 5,
                'search_radius': 0.007,
            },
            {
                'descriptor': 'filter_by_size',
                'cluster_size': [40, 10000000],
                'max_clouds_returned': 100,
                'x_axis': [0.055, 0.095],
                'y_axis': [0.065, 0.095],
                'z_axis': [0, 0],
            },
            {
                'descriptor': 'sort_clouds_distance',
                'close_to_far': True,
            },
            {
                'descriptor': 'find_pose',
                'rotate_z': True # Rotates EVERYTHING
            },
        ]
    } , 
}