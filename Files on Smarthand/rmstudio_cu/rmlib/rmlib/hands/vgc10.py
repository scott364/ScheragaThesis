class VGC10():   
    def __init__(self, vgc10_config, robot):
        self.connected = True
        self.robot = robot
        self.CAN_SET_WIDTH = False
        
    def grip(self):
        self.robot.arm.set_tool_digital_outputs(True, True)
        
    def release(self):
        self.robot.arm.set_tool_digital_outputs(False, False)
        
        
    
        
    