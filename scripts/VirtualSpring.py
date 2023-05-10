import math
import numpy as np
from ObstacleHelpers import closestPointOnLine
import configparser
import os, rospkg

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def shortest_angle(a1, a2):
  a = a1 - a2
  return (a + np.pi) % (2*np.pi) - np.pi

def rotate(p, origin, angle):
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    o = np.atleast_2d(origin)
    p = np.atleast_2d(p)
    return np.squeeze((R @ (p.T-o.T) + o.T).T)

class VirtualSpring:
  def __init__(self, desired_follow_distance, config_file="", desired_distance_range=[.25, .25], desired_angle_range=[np.pi/12, np.pi/12], desired_follow_angle=0, vel_cap=1.4, predict=True) -> None:
    self.static_obstacles = []; # Position array of static obstacles
    self.human_trajectory = []; #Full trajectory array for followed agent
    self.human_velocity = []; #Stored human velocity vector
    self.human_angle = 0 #Last recorded human angle, currently set by the angle of the velocity

    self.robot_position = []; #Current robot position 
    self.robot_velocity = []; #Current robot velocity

    self.desired_follow_angle = desired_follow_angle #Desired angle to follow, 0 is directly in front
    self.desired_follow_distance = desired_follow_distance #Desired distance to follow the human
    self.desired_angle_range = desired_angle_range
    self.desired_distance_range = desired_distance_range
    self.vel_cap = vel_cap
    self.predict = predict

    if(config_file != ""):
      rospack = rospkg.RosPack()
      Config = configparser.ConfigParser()
      try:
        Config.read(os.path.join(rospack.get_path("human_following"), 'scripts', config_file))
        self.HUMAN_K_VALUE = float(Config['SinkValues']['OutsideDesired']) #The k-value of the virtual spring attached to the human
        self.HUMAN_BOUND_K_VALUE = float(Config['SinkValues']['Desired']) #The k-value of the virtual spring attached to the human
        self.OBSTACLE_K_VALUE = float(Config['SourceValues']['Obstacles']) #The k-value of the virtual spring attached to the obstacles
        self.OCCLUSION_CONSTANT = float(Config['SourceValues']['Occlusion']) #The k-value of the virtual spring attached to the occlusion lines
        self.HUMAN_REPULSIVE = float(Config['SourceValues']['Human']) #The k-value of the source the pushes the robot away from the human
      except:
        print("Invalid Configuration File")
    else:
      self.HUMAN_K_VALUE = .9 #The k-value of the virtual spring attached to the human
      self.HUMAN_BOUND_K_VALUE = 0.2 #The k-value of the virtual spring attached to the human
      self.OBSTACLE_K_VALUE = .4 #The k-value of the virtual spring attached to the obstacles
      self.OCCLUSION_CONSTANT = .8 #The k-value of the virtual spring attached to the occlusion lines
      self.HUMAN_REPULSIVE = .4 #The k-value of the source the pushes the robot away from the human

    pass

  #Predicts a location for a human time_step into the future using a dynamics model on the previous human locations
  def human_prediction(self, n=10):
    if len(self.human_trajectory) < 2:
        return self.human_trajectory[-1]
    current_human_vel = self.human_trajectory[-1] - self.human_trajectory[-2]
    return self.human_trajectory[-1] + (current_human_vel * n)

  #Defines a repulsive force given a point, radius, and spring constant
  def repulsive_force(self, point, circle_center, radius, spring_k):
    distance = math.sqrt((point[0] - circle_center[0])**2 + (point[1] - circle_center[1])**2)
    angle = math.atan2(point[1] - circle_center[1], point[0] - circle_center[0])
    force_magnitude = 1 / abs(distance - radius)
    force_x = force_magnitude * math.cos(angle) * spring_k
    force_y = force_magnitude * math.sin(angle) * spring_k
    return (force_x, force_y)

  def updateHuman(self, human_position):
    self.human_trajectory.append(np.array(human_position))
    if(len(self.human_trajectory) > 50):
      self.human_trajectory = self.human_trajectory[-50:]
    self.human_angle = human_position[2]

  def updateObstacles(self, obstacles):
    #Update obstacle array
    self.static_obstacles = obstacles
    
  def updateRobot(self, robot_position):
    self.robot_position = np.array(robot_position)
    pass
  
  #Returns the percentage occluded of a +-60 degree angle
  def getOccluded(self):
    if len(self.human_trajectory) == 0 or len(self.robot_position) == 0:
      return 0
    obstacles = self.static_obstacles
    human = self.human_trajectory[-1]
    if(self.predict):
        human = self.human_prediction()
    lines = []

    occlusion = 0
    for obstacle in obstacles:
            
        obstacle_type = obstacle['type']

        obstacle_pos = obstacle['position']
        if(obstacle_type == 'line'):
            for angle in np.linspace(-np.pi/3, np.pi/3, 10):
                rotated_point = rotate(human[0:2], self.robot_position, angle)
                if intersect(obstacle_pos[0], obstacle_pos[1], rotated_point, self.robot_position):
                    occlusion += 1
    return occlusion
  
  #Creates a line of occlusion between the human and the closest point of each obstacles to the robot
  def calc_occlusion_lines(self, human_position):
    obstacles = self.static_obstacles

    lines = []

    for obstacle in obstacles:
      obstacle_type = obstacle['type']

      obstacle_pos = obstacle['position']
      if(obstacle_type == 'line'):
        diff1 = self.dist(obstacle_pos[0], human_position[0:2])
        diff2 = self.dist(obstacle_pos[1], human_position[0:2])
        if diff1 > diff2:
          lines.append(np.array([obstacle_pos[0], obstacle_pos[1]]))
        else:
          lines.append(np.array([obstacle_pos[1], obstacle_pos[0]]))
    return lines
        
  def dist(self, a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

  #For use in calculating the variance when running A*
  def getDesired(self):
    if len(self.human_trajectory) == 0:
      return [0, 0]
    human = self.human_trajectory[-1]
    angle_2 = 2* np.pi - self.desired_follow_angle
    desired_position2 = [human[0] + (self.desired_follow_distance * math.sin(angle_2 + self.human_angle)), human[1] + (self.desired_follow_distance * math.cos(angle_2 + self.human_angle))]
    desired_position = [human[0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), human[1] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]
    if self.dist(self.robot_position, desired_position2) < self.dist(self.robot_position, desired_position):
      return desired_position2
    else:
      return desired_position
  
  def getRobotControlVelocity(self, point=None):
    if len(self.human_trajectory) == 0:
      return [0, 0]
    human = self.human_trajectory[-1]
    if(self.predict):
        human = self.human_prediction()
    desired_position = point
    desired_position2 = point
    if point == None:
      desired_position = [human[0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), human[1] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]
      angle_2 = 2* np.pi - self.desired_follow_angle
      desired_position2 = [human[0] + (self.desired_follow_distance * math.sin(angle_2 + self.human_angle)), human[1] + (self.desired_follow_distance * math.cos(angle_2 + self.human_angle))]
    desired_angle = self.desired_follow_angle
    angle_2 = 2* np.pi - self.desired_follow_angle
    robot_velocity = np.array([0.0, 0.0])


    occlusion_avoidance_force = np.array([0.0, 0.0])
    hit = False
    #Handle the observed obstacles
    for obstacle in self.static_obstacles:
      obstacle_type = obstacle['type']

      obstacle_pos = obstacle['position']

      if(obstacle_type == 'circle'):
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, obstacle_pos, obstacle_pos[2], self.OBSTACLE_K_VALUE)))
      elif(obstacle_type == 'line'):
        closest_point_on_line = np.array(closestPointOnLine(obstacle_pos[0], obstacle_pos[1], self.robot_position))
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, closest_point_on_line, 0.5, self.OBSTACLE_K_VALUE)))
        #If there is occlusion between the human and both mirrored points, set the desired position to back following
        if intersect(obstacle_pos[0], obstacle_pos[1], human, desired_position2) and intersect(obstacle_pos[0], obstacle_pos[1], human, desired_position):
          desired_position = human
          desired_position2 = human
          angle_2 = 0
          desired_angle = 0
        elif intersect(obstacle_pos[0], obstacle_pos[1], human, desired_position2):
          desired_position2 = desired_position
          angle_2 = desired_angle
        elif intersect(obstacle_pos[0], obstacle_pos[1], human, desired_position):
          desired_position = desired_position2
          desired_angle = angle_2
        
        if intersect(obstacle_pos[0], obstacle_pos[1], human, self.robot_position):
          hit = True
          closest_end_point = obstacle_pos[0] if self.dist(obstacle_pos[0], self.robot_position) < self.dist(obstacle_pos[1], self.robot_position) else obstacle_pos[1]
          if (closest_point_on_line[0] != closest_end_point[0]) or closest_point_on_line[1] != closest_end_point[1]:
            force = (closest_end_point - closest_point_on_line) / np.linalg.norm(closest_end_point - closest_point_on_line) * self.vel_cap * 1.5
            robot_velocity += force
          else:
            further_point = obstacle_pos[1] if self.dist(obstacle_pos[0], self.robot_position) < self.dist(obstacle_pos[1], self.robot_position) else obstacle_pos[0]
            force = (closest_end_point - np.array(further_point)) / np.linalg.norm(closest_end_point -  np.array(further_point)) * self.vel_cap * 1
            robot_velocity += force

        #Create an occlusion avoidance force in the direction between the object and the human
        elif not hit and self.dist(closest_point_on_line, self.robot_position) < 1:
          diff1 = closest_point_on_line - human[0:2]
          force = ((((closest_point_on_line) + diff1 * 100) - closest_point_on_line) / np.linalg.norm(np.array([closest_point_on_line, (closest_point_on_line) + diff1 * 100] )) * -1 / self.dist(closest_point_on_line, self.robot_position) * self.OCCLUSION_CONSTANT)
          occlusion_avoidance_force += force
    if not hit:
      robot_velocity += occlusion_avoidance_force

    robot_velocity += np.array(list(self.repulsive_force(self.robot_position, human, .5, self.HUMAN_REPULSIVE)))
    
    #If the robot is within the desired  range apply very little force towards the desired:
    if ((shortest_angle(desired_angle, np.arctan2(*(self.robot_position - human[0:2])) - human[2]) < self.desired_angle_range[1] and  shortest_angle(desired_angle, np.arctan2(*(self.robot_position - human[0:2])) - human[2]) > -self.desired_angle_range[0]) or (shortest_angle(angle_2, np.arctan2(*(self.robot_position - human[0:2])) - human[2]) < self.desired_angle_range[0] and  shortest_angle(angle_2, np.arctan2(*(self.robot_position - human[0:2])) - human[2]) > -self.desired_angle_range[1])) and (self.desired_follow_distance - self.desired_distance_range[0] < self.dist(human, self.robot_position) < self.desired_follow_distance + self.desired_distance_range[1]):
      print('In Bound')
      vel1 = np.array(list(map(lambda x: self.HUMAN_BOUND_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position))))
      vel2 = np.array(list(map(lambda x: self.HUMAN_BOUND_K_VALUE * (x[0] - x[1]), zip(desired_position2, self.robot_position))))
      
      #Pick the point with the higher vel/dist^2 ratio
      robot_attractive = vel1 if np.linalg.norm(vel1 + robot_velocity)/(self.dist(desired_position, self.robot_position))**2 > np.linalg.norm(vel2 + robot_velocity)/(self.dist(desired_position2, self.robot_position))**2 else vel2
      robot_attractive = robot_attractive if np.linalg.norm(robot_attractive) < self.vel_cap else robot_attractive / np.linalg.norm(robot_attractive) * self.vel_cap
      robot_velocity += robot_attractive
    else:
      vel1 = np.array(list(map(lambda x: self.HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position))))
      vel2 = np.array(list(map(lambda x: self.HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position2, self.robot_position))))
      #Pick the point with the higher vel/dist^2 ratio
      robot_attractive = vel1 if np.linalg.norm(vel1 + robot_velocity)/(self.dist(desired_position, self.robot_position))**2 > np.linalg.norm(vel2 + robot_velocity)/(self.dist(desired_position2, self.robot_position))**2 else vel2
      robot_attractive = robot_attractive if np.linalg.norm(robot_attractive) < self.vel_cap else robot_attractive / np.linalg.norm(robot_attractive) * self.vel_cap
      robot_velocity += robot_attractive
    
    #Human deadzone so the robot doesn't run into the human
    if (self.dist(self.robot_position, human) < self.desired_follow_distance/2):
      robot_velocity *= 0
    robot_velocity = robot_velocity if np.linalg.norm(robot_velocity) < self.vel_cap else robot_velocity / np.linalg.norm(robot_velocity) * self.vel_cap
    return list(robot_velocity)




