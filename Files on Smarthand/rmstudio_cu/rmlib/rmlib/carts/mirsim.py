import requests
import json
import numpy as np
import time
import math
import random
import uuid
 
class Mir_Cart: 
    def __init__(self, mir_config, _):
        self.connected = True
        self.start_mission_time = 0
        self.MISSION_SIM_TIME = 5
        return 

    def get_status(self):
        distance_to_next_target = self.MISSION_SIM_TIME - (time.time() - self.start_mission_time)
        if distance_to_next_target < 0:
            state_text = 'Ready'
        else:
            state_text = 'Running'
            
        
        status_info = {
            'allowed_methods': None,
            'battery_percentage': -1.0,
            'battery_time_remaining': -1,
            'distance_to_next_target': distance_to_next_target,
            'errors': [],
            'footprint': '[[0.506,-0.32],[0.506,0.32],[-0.454,0.32],[-0.454,-0.32]]',
            'joystick_low_speed_mode_enabled': False,
            'joystick_web_session_id': '',
            'map_id': 'a7524472-22a1-11ea-aeda-94c691a739e9',
            'mission_queue_id': None,
            'mission_queue_url': None,
            'mission_text': 'Cameras are ready to stream',
            'mode_id': 7,
            'mode_key_state': 'idle',
            'mode_text': 'Mission',
            'moved': 138375.33,
            'position': {'orientation': -67.43964385986328,
            'x': 47.57450866699219,
            'y': 15.078078269958496},
            'robot_model': 'MiR100',
            'robot_name': 'MiR_R1319',
            'safety_system_muted': False,
            'serial_number': '190100003001319',
            'session_id': 'a191c2cf-22a1-11ea-aeda-94c691a739e9',
            'state_id': 10,
            'state_text': state_text,
            'unloaded_map_changes': False,
            'uptime': 8901,
            'user_prompt': None,
            'velocity': {'angular': 0.0, 'linear': 0.0}
        }
        return status_info
    
    def get_status_json(self):
        #Shows the robot's battery level, mission status and current state (eg. paused, executing)
        status_info = self.get_status()
        status_info = json.dumps(status_info)
        return status_info
    
    def get_state(self):
        status = self.get_status()
        try: 
            rtn = status['state_text']
        except:
            rtn = self.get_state()
        return rtn
    
    def is_mission_running(self):
        cart_state = self.get_state()
        if cart_state not in ['Ready','Error']:
            return True
        else:
            return False
        
    def get_positions(self):
        info = [
            {
                'guid': '84039e4a-22b1-11ea-aeda-94c691a739e9',
                'map': '/v2.0.0/maps/a7524472-22a1-11ea-aeda-94c691a739e9',
                'name': '1',
                'type_id': 0,
                'url': '/v2.0.0/positions/84039e4a-22b1-11ea-aeda-94c691a739e9'
            },
            {
                'guid': '8b761f49-22b1-11ea-aeda-94c691a739e9',
                'map': '/v2.0.0/maps/a7524472-22a1-11ea-aeda-94c691a739e9',
                'name': '2',
                'type_id': 0,
                'url': '/v2.0.0/positions/8b761f49-22b1-11ea-aeda-94c691a739e9'
            },
            {
                'guid': '9062c128-22b1-11ea-aeda-94c691a739e9',
                'map': '/v2.0.0/maps/a7524472-22a1-11ea-aeda-94c691a739e9',
                'name': '3',
                'type_id': 0,
                'url': '/v2.0.0/positions/9062c128-22b1-11ea-aeda-94c691a739e9'
            },
        ]
        return info
    
    def get_missions(self):
        info = [
            {
                'guid': 'mirconst-guid-0000-0001-actionlist00',
                'name': 'Move',
                'url': '/v2.0.0/missions/mirconst-guid-0000-0001-actionlist00'
            },
            {
                'guid': 'mirconst-guid-0000-0003-actionlist00',
                'name': 'GoToPositionPrototype',
                'url': '/v2.0.0/missions/mirconst-guid-0000-0003-actionlist00'
            },
            {
                'guid': 'mirconst-guid-0000-0004-actionlist00',
                'name': 'ChargeAtStation',
                'url': '/v2.0.0/missions/mirconst-guid-0000-0004-actionlist00'
            },
               ]
        return info
    
    def get_mission(self, mission_guid):
        info = {
            'guid': 'mirconst-guid-0000-0001-actionlist00',
            'name': 'Move',
            'url': '/v2.0.0/missions/mirconst-guid-0000-0001-actionlist00'
        },
        return info
    
    def delete_mission(self, mission_guid):  
        return True
        
    def add_mission(self, guid=str(uuid.uuid1()), name='rm_mission', group_id="mirconst-guid-0000-0011-missiongroup"):
        return True
        
    def add_action(self, mission_id, action):
        return True
    
    def add_mission_to_queue(self, mission_id):
        return True
    
    def clear_mission_queue(self):
        return True
        
    def start_mission(self):
        return True
        
    def run_rm_mission(self, actions):
        return True

    def get_sounds(self):
        info = [
            {
                'guid': 'mirconst-guid-0000-0001-sounds000000',
                'length': '0:00:11',
                'name': 'Beep',
                'url': '/v2.0.0/sounds/mirconst-guid-0000-0001-sounds000000',
                'volume': 100
            },
            {
                'guid': 'mirconst-guid-0000-0002-sounds000000',
                'length': '0:00:07',
                'name': 'Horn',
                'url': '/v2.0.0/sounds/mirconst-guid-0000-0002-sounds000000',
                'volume': 100
            },
            {
                'guid': 'mirconst-guid-0000-0003-sounds000000',
                'length': '0:00:07',
                'name': 'Foghorn',
                'url': '/v2.0.0/sounds/mirconst-guid-0000-0003-sounds000000',
                'volume': 50
            },
        ]
        return info
    
    def play_sound(self, sound_id, volume=100):
        return True
    
    def update_localization(self):
        return True
      
    def dummy_stop(self):
        return False 
    
    def wait_until_finished(self, stop_condition='dummy'):
        if stop_condition == 'dummy':
            stop_condition = self.dummy_stop
        time_start = time.time()
        
        # Wait for mission to start
        while(not self.is_mission_running()):
            if time.time()-time_start > 5:
                break
        while(self.is_mission_running()):
            if stop_condition():
                #Stop Cart Here
                break
        if self.get_state() != 'Ready':
            raise RuntimeError("Cart Error")
            
    def get_coordinates(self):
        return (47.57450866699219, 15.078078269958496, -157.43964385986328)

    def move_to_coordinates(self, x, y, yaw, blocking=True):
        self.start_mission_time = time.time()
        return True
    
    def get_position_coordinates(self, position_id):
        return (47.57450866699219, 15.078078269958496, -157.43964385986328)
    
    def move_to_position(self, position_id, blocking=True):
        coordinates = self.get_position_coordinates(position_id)
        self.move_to_coordinates(coordinates[0], coordinates[1], coordinates[2], blocking=blocking)
        return True
    
    def get_position_by_name(self, position_name):
        positions = self.get_positions()
        for position in positions:
            if position['name'] == position_name:
                break
        return position['guid']
                

        
        
        
        
        
        
    
    
    
    
    
    
