robot_config = {
    'active_components' : {
        'camera' : [],
        'hand' : [],
        'arm' : ['sim_ur5'],
        'cart' : ['mir_100'],
    },
    'my_components' : {
        'mir_100' : {
            "module_name" : "carts.mir100",
            "class_name" : "Mir_Cart",
            "ip_address" : '10.1.12.186'
        },
        "sim_mir_100" : 
        {
            "module_name" : "carts.mirsim",
            "class_name" : "Mir_Cart"
        },
        "sim_ur5" : 
        {
            "module_name" : "arms.ursim",
            "class_name" : "URSim"
        },
        "ur5" : 
        {
            "module_name" : "arms.ur5",
            "class_name" : "UR5"
        },
        'd415' : {
            'module_name' : 'cameras.realsense',
            'class_name' : 'RealSense',
            'camera_model' : 'd415',
            'ci_cam_offset' : [-0.0179, -0.0753, -0.02],
            'pc_cam_offset' : [-0.0179, -0.0753, -0.02],
        },
        'd410' : {
            'module_name' : 'cameras.realsense',
            'class_name' : 'RealSense',
            'camera_model' : 'd410',
            'ci_cam_offset' : [-0.0179, -0.0753, -0.02],
            'pc_cam_offset' : [-0.0179, -0.0753, -0.02],
        },
    },
}