robotConfig = {
    'active_components' : {
        'camera' : ['cam_config'],
        'hand' : ['hand_config'],
        'arm' : ['arm_config'],
        'ft' : ['ft_config'],
    },
    'my_components' : {
        'cam_config' : {
            'module_name' : 'cameras.realsense',
            'class_name' : 'RealSense',
            'camera_model' : 'd410', #'d415',
            'ci_cam_offset' : [-0.036, -0.028, -0.079], #[-0.0179, -0.0753, -0.02], # [-0.036, -0.028, -0.079]
            # self.tcp_to_camera_pose = self.translate_pose(np.eye(4), x=-0.037, y=-0.033, z=-0.079-self.finger_length)
            'pc_cam_offset' : [ -0.037 , -0.033 , -0.079 ],
        },    
        'arm_config' : {
            "module_name" : "arms.ur5",
            "class_name" : "UR5",
            "ip_address" : "192.168.0.101", 
            "xmlrpc_port" : "8003",
            "max_linear_speed" : 0.25,
            "max_linear_accel" : 1.2,
            "max_joint_speed" : 1.05,
            "max_joint_accel" : 1.4,
            "default_linear_speed" : 0.1,
            "default_joint_speed" : 0.7,
            "default_linear_accel" : 0.8,
            "default_joint_accel" : 0.8
        },
        'ft_config' : {
            "module_name" : "sensors.ftsensor_optoforce",
            "class_name"  : "OptoForce",
            "ip_address"  : "192.168.0.100",#"10.1.12.183",
            "dataHz"      : 50.0,
            "filter"      : {
                "alpha" : 0.012 ,
                "beta"  : 0.035 ,
            },  
            "remote_ip" : "", #"0.0.0.0", #"localhost", #
            "remote_port" : 10000,
            "local_ip" : "127.0.0.1", #"0.0.0.0", #"localhost", #
            "local_port" : 20000,
            "max_workers" : 10,
        },
        'hand_config' : {
            "module_name" : "hands.smarthand",
            "class_name" : "SmartHand",
            "finger_length": 0.0415,
            "finger_width_outer" : 0.015,
            "finger_width_inner" : 0.0,
            "finger_depth" : 0.014
        },
    },
}