from math import pi, acos, atan2, sqrt, sin, asin, degrees, cos

class RobotArm:
    def __init__(self):
        self.l1 = 0.065
        self.l2 = 0.101
        self.l3 = 0.094
        self.l4 = 0.169
        self.min_angle = -120
        self.max_angle = 120
        self.min_value = 0
        self.max_value = 24000
        
    def inverse_kinematics(self, pose):
        x, y, z = pose
        y = -y
        # x+=0.03
        y+=0
        z+=0.0
        xi = 0
        theta6 = atan2(y, x)
        delta =sqrt(x**2 +y**2)-self.l2-self.l3
    
        if delta >0:
            try:
                xi = asin(delta/self.l4)+ 0.2
                print(delta)
            except:
                pass
        z  = z -self.l1+self.l4*cos(xi)
        x = sqrt(x**2 + y**2)-(self.l4+0.03)*sin(xi)
        r = sqrt(x**2 + z**2)
        print((self.l2**2 + self.l3**2 - r**2) / (2 * self.l2 * self.l3))
        theta4 = pi - acos(round((self.l2**2 + self.l3**2 - r**2) / (2 * self.l2 * self.l3),4))
        #print(theta4)
        alpha = atan2(x, z)### this one might need to be changed
        beta = asin((self.l3 * sin(theta4)) / r)
        theta5 = pi/2+alpha - beta 
        if theta4 < pi/4:
            theta3 = 2*pi-theta4 - theta5-xi
        elif theta4+theta5<pi:
            theta3 = pi -theta4 - theta5-xi
        elif theta4+theta5<2*pi:
            theta3 = 2*pi - theta4 - theta5-xi
        
        theta3 = 21000-self.rad_to_centidegrees(theta3)
        theta4 = 12000+self.rad_to_centidegrees(theta4)
        theta5 = 21000-self.rad_to_centidegrees(theta5)
        theta6 = 12000-self.rad_to_centidegrees(theta6)
        if theta3 > 12000 and theta4 < 12000:
            theta4 = 12000
            theta3 = 12000
        servo_angles = [theta3, theta4, theta5, theta6]
        return servo_angles

    '''def convert_to_centidegrees(self, servo3_angle, servo4_angle, servo5_angle, servo6_angle):
        servo3_angle = min(self.max_angle, max(self.min_angle, servo3_angle))
        servo4_angle = min(self.max_angle, max(self.min_angle, self.max_angle - servo4_angle + self.min_angle))
        servo5_angle = min(self.max_angle, max(self.min_angle, servo5_angle))
        servo6_angle = min(self.max_angle, max(self.min_angle, servo6_angle))
        
        servo3_angle_centi = int((servo3_angle - self.min_angle) / (self.max_angle - self.min_angle) * (self.max_value - self.min_value) + self.min_value)
        servo4_angle_centi = int((servo4_angle - self.min_angle) / (self.max_angle - self.min_angle) * (self.max_value - self.min_value) + self.min_value)
        servo5_angle_centi = int((servo5_angle - self.min_angle) / (self.max_angle - self.min_angle) * (self.max_value - self.min_value) + self.min_value)
        servo6_angle_centi = int((servo6_angle - self.min_angle) / (self.max_angle - self.min_angle) * (self.max_value - self.min_value) + self.min_value)
        
        return servo3_angle_centi, servo4_angle_centi, servo5_angle_centi, servo6_angle_centi
        '''
    def rad_to_centidegrees(self, rad):
        return int(rad * 180 / pi * 100)


