import requests
import json
import numpy as np
import time
import math
import uuid
 
class Mir_Cart: 
    def __init__(self, mir_config, _):
        self.mobile_platform_ip = mir_config['ip_address']        
        self.headers = {
            "Accept-Language":"en_US", 
            "Authorization":"Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.status_url = "http://"+self.mobile_platform_ip+"/api/v2.0.0/status"
        self.positions_url = "http://"+self.mobile_platform_ip+"/api/v2.0.0/positions"

        try:
            self.connected = True
        except:
            self.connected = False
            print("Error Connecting to Cart at IP Address: " + self.mobile_platform_ip)

        return 
    
    
    def get_status(self):
        #Shows the robot's battery level, mission status and current state (eg. paused, executing)
        q = requests.get(self.status_url, headers = self.headers)
        status_info = q.json()
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
        status = self.get_status()
        if 'map_id' in status.keys():
            map_id = status['map_id']
        else: 
            print('Cart return status', status)
            return None
        
        map_id = status['map_id']
        q = requests.get("http://"+self.mobile_platform_ip+"/api/v2.0.0/maps/"+ map_id +"/positions", headers = self.headers)
        info = q.json()  
        return info
    
    def get_missions(self):
        q = requests.get("http://"+self.mobile_platform_ip+"/api/v2.0.0/missions", headers = self.headers)
        info = q.json() 
        return info
    
    def get_mission(self, mission_guid):
        q = requests.get("http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/"+mission_guid, headers = self.headers)
        info = q.json() 
        return info
    
    def delete_mission(self, mission_guid):
        q = requests.delete("http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/"+mission_guid, headers = self.headers) 
        return q
        
    def add_mission(self, guid=str(uuid.uuid1()), name='rm_mission', group_id="mirconst-guid-0000-0011-missiongroup"):
        mission = { 
            "guid" : guid,
            "name": name, 
            "group_id": group_id,
        }
        
        q = requests.post("http://"+self.mobile_platform_ip+"/api/v2.0.0/missions", headers = self.headers, json = mission)
        return q
        
    def add_action(self, mission_id, action):
        url = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions" %(mission_id)
        r = requests.post(url, headers = self.headers, json = action)
        return r
    
    def add_mission_to_queue(self, mission_id):
        mission_id_dict = {"mission_id": mission_id}
        q = requests.post("http://"+self.mobile_platform_ip+"/api/v2.0.0/mission_queue", headers = self.headers, json = mission_id_dict)
        return q
    
    def clear_mission_queue(self):
        q = requests.delete('http://'+self.mobile_platform_ip+'/api/v2.0.0/mission_queue', headers = self.headers)
        
    def start_mission(self):
        q = requests.put(self.status_url, headers = self.headers, json = {"clear_error": True} )
        q = requests.put(self.status_url, headers = self.headers, json = {"state_id": 3})
        return
        
    def run_rm_mission(self, actions):
        guid = '000000000000000000000000000000000001'
        
        # Clear mission queue
        self.clear_mission_queue()
        
        # Delete old mission
        self.delete_mission(guid)
        # Add new mission
        self.add_mission(guid=guid)
        for action in actions:
            print(self.add_action(mission_id=guid, action=action))
 
        # Add to mission queue
        self.add_mission_to_queue(guid)
        
        # Start mission
        self.start_mission()

    def get_sounds(self):
        q = requests.get("http://"+self.mobile_platform_ip+"/api/v2.0.0/sounds", headers = self.headers)
        info = q.json() 
        return info
    
    def play_sound(self, sound_id, volume=15):
        action = {
            "action_type": "sound",
            "priority": 0,
            "parameters": [
              {"id": "sound", "value": sound_id},
              {"id": "volume", "value": volume},
              {"id": "mode", "value": "full"},
              {"id": "duration","value": "00:00:01.000000"}
            ], 
        } 
        self.run_rm_mission([action])
    
    def update_localization(self):
        '''
        Updates the robot's position in the map
        Should be used in areas with lots of map features
        ''' 
        mission_queue = {"mission_id": self.mission_id_localize}
        
        q = requests.put(self.status_url, headers = self.headers, json = self.clear_error)
        q = requests.put(self.status_url, headers = self.headers, json = self.robot_ready)
        q = requests.post(self.mission_queue_url, headers = self.headers, json = mission_queue)

        return
      
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
        #Shows the robot's current x position, y position, orientation and velocity
        status = self.get_status()
        return status['position']['x'], status['position']['y'], status['position']['orientation']-90

    def move_to_coordinates(self, x, y, yaw, blocking=True):
        '''
        Moves the robot to a coordinate on the map        
        '''        
        action = {
            "action_type": "move_to_position", 
            "priority": 0, 
            "parameters": [
                {"id": "x", "value": x}, 
                {"id": "y", "value": y}, 
                {"id": "orientation", "value": yaw}, 
                {"id": "retries", "value": 5}, 
                {"id": "distance_threshold", "value": 0.1}
            ]
        }

        self.run_rm_mission([action])
        
        if blocking:
            self.wait_until_finished()
        return
    
    def get_position_coordinates(self, position_id):
        print('Position ID:', position_id)
        url = self.positions_url + "/" + position_id
        q = requests.get(url, headers = self.headers)
        info = q.json() 
        print('Info:', info)
        return [info['pos_x'], info['pos_y'], info['orientation']]
    
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
                

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#     def relative_move(self, forwards=0, left=0, yaw=0, planning=True):
#         '''
#         Moves the robot relative to its current position in meters/degrees
#         Setting the planning variable to 0 disables path planning (The robot moves slowly in this mode and can be used when unloading from vehicles/leaving docking stations)
#         Positive yaw values rotate the robot CCW
#         '''
#         url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" %(self.mission_id_moveToCoordinate, self.action_id_moveToCoordinate)
#         url_2 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" %(self.mission_id_relativeMove, self.action_id_relativeMove)

#         q = requests.put(self.status_url, headers = self.headers, json = self.clear_error)
#         q = requests.put(self.status_url, headers = self.headers, json = self.robot_ready)

#         if planning is False:
#             relative_move = {
#                 "action_type": "relative_move", 
#                 "mission_id": self.mission_id_relativeMove, 
#                 "priority": 0, 
#                 "guid": self.action_id_relativeMove,
#                 "parameters": [
#                     {"id": "x", "value": forwards}, 
#                     {"id": "y", "value": left}, 
#                     {"id": "orientation", "value": yaw}, 
#                     {"id": "max_linear_speed", "value": 0.5},
#                     {"id": "max_angular_speed", "value": 1.0}, 
#                     {"id": "collision_detection", "value": True}
#                 ]
#             }

#             mission_queue = {"mission_id": self.mission_id_relativeMove}

#             q = requests.put(url_2, headers = self.headers, json = relative_move)
#             q = requests.post(self.mission_queue_url, headers = self.headers, json = mission_queue)


#         if planning is True:
#             #Convert the relative movement requests into a new coordinate
#             q = requests.get(self.status_url, headers = self.headers)
#             status_info = q.json()
#             for a in status_info:
#                 if a == "position":
#                     pose = np.array([status_info[a][u'x'], status_info[a][u'y'], status_info[a][u'orientation']])

#             X = forwards*np.cos(np.deg2rad(pose[2])) + left*np.sin(np.deg2rad(pose[2])) + pose[0]
#             Y = forwards*np.sin(np.deg2rad(pose[2])) + left*np.cos(np.deg2rad(pose[2])) + pose[1]
#             Yaw = yaw + pose[2]

#             if Yaw < -180:
#                 Yaw = Yaw + 360
#             if Yaw > 180:
#                 Yaw = Yaw - 360

#             move_to_coordinate = {
#                 "action_type": "move_to_position", 
#                 "mission_id": self.mission_id_moveToCoordinate, 
#                 "priority": 0, 
#                 "guid": self.action_id_moveToCoordinate,
#                 "parameters": [
#                     {"id": "x", "value": X}, 
#                     {"id": "y", "value": Y},                    
#                     {"id": "orientation", "value": Yaw}, 
#                     {"id": "retries", "value": 5},                        
#                     {"id": "distance_threshold", "value": 0.1}
#                 ]
#             }

#             mission_queue = {"mission_id": self.mission_id_moveToCoordinate}

#             q = requests.put(url_1, headers = self.headers, json = move_to_coordinate)
#             q = requests.post(self.mission_queue_url, headers = self.headers, json = mission_queue)

#         return 
    
#     def move_to_pose(self, pose_mtrx):
#         pose_vec = self.pose_mtrx_to_vec(pose_mtrx)
#         x = pose_vec[0]
#         y = pose_vec[1]
#         yaw = math.degrees(pose_vec[5])+90
#         self.move_to_coordinate(x, y, yaw)
#         return


#     def get_pose(self):
#         #Shows the robot's current x position, y position, orientation and velocity
#         q = requests.get(self.status_url, headers = self.headers)

#         status_info = q.json()
#         pose = self.origin_pose()
#         pose = self.translate_pose(pose, x=status_info['position']['x'], y=status_info['position']['y'], frame='self')
#         pose = self.rotate_pose(pose, rz=math.radians(status_info['position']['orientation']-90), frame='self')
#         return pose
    
#     def get_coordinates(self):
#         #Shows the robot's current x position, y position, orientation and velocity
#         q = requests.get(self.status_url, headers = self.headers)

#         status_info = q.json()
#         return [status_info['position']['x'], status_info['position']['y'], status_info['position']['orientation']-90]
    
#     def get_position_pose(self, position_name):
#         coordinates = self.mp_get_position_coordinates(position_name)
#         pose = self.origin_pose()
#         pose = self.translate_pose(pose, x=coordinates[0], y=coordinates[1], frame='self')
#         pose = self.rotate_pose(pose, rz=math.radians(coordinates[2]-90), frame='self')
#         return pose
    
#     def set_lighting(self, time, speed, intensity, effect, color_1, color_2):
#         '''
#         Controls the robot lighting

#         time_format(str): "00:00:00"
#         effect(str): "solid", "blink", "fade", "chase", "wave", "rainbow" 
#         speed(str): "fast", "slow"
#         intensity(int): 0-100
#         color_1(str), color_2(str): Red: "#ff0000", Green: "#00ff00", 
#             Blue: "#0000ff", White: "#ffffff", Black: "#000000", Yellow: "#ffff00",
#             Magenta: "#ff00ff", Cyan: "#00ffff", Orange: "#ffa500", Pink: "#ffc0cb"
            
#         **Black and Pink aren't very noticeable

#         If the "solid" effect is used, only color_1 is used
#         If the "rainbow" effect is used, neither provided color is used
#         '''
#         url_1 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" %(self.mission_id_lighting, self.action_id_lighting_type)
#         url_2 = "http://"+self.mobile_platform_ip+"/api/v2.0.0/missions/%s/actions/%s" %(self.mission_id_lighting, self.action_id_lighting_duration)

#         light_type = {
#             "action_type": "light", 
#             "mission_id": self.mission_id_lighting, 
#             "priority": 0,
#             "parameters": [
#                 {"id": "speed", "value": speed }, 
#                 {"id": "light_effect", "value": effect},
#                 {"id": "color_1", "value": color_1}, 
#                 {"id": "color_2", "value": color_2},
#                 {"id": "intensity", "value": 100}
#             ]
#         }  

#         light_time = {
#             "action_type": "wait", 
#             "mission_id": self.mission_id_lighting, 
#             "priority": 1,
#             "parameters": [{"id": "time", "value": time }]
#         } 

#         mission_queue = {"mission_id": self.mission_id_lighting}

#         q = requests.put(self.status_url, headers = self.headers, json = self.clear_error)
#         q = requests.put(self.status_url, headers = self.headers, json = self.robot_ready)
#         q = requests.put(url_1, headers = self.headers, json = light_type)
#         q = requests.put(url_2, headers = self.headers, json = light_time)
#         q = requests.post(self.mission_queue_url, headers = self.headers, json = mission_queue)
        
#         return 
    

        