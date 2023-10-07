import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

listOfPoints = np.array([np.array([0.0, 0.0]), np.array([5.0, 10.0])])
listOfCircles = np.array([[[4, 3], math.sqrt(5)], [[0.25, 4.05], math.sqrt(5)], [[7, 8], 1]])
roboradius = 0.5
index = 1

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def lineIntersectVector(pointOne, pointTwo, pointThree, pointFour):
    vNum12 = np.linalg.det(np.array([[pointOne[0], pointOne[1]],[pointTwo[0], pointTwo[1]]]))
    vNum34 = np.linalg.det(np.array([[pointThree[0], pointThree[1]],[pointFour[0], pointFour[1]]]))
    vDenomx12 = np.linalg.det(np.array([[pointOne[0], 1], [pointTwo[0], 1]]))
    vDenomx34 = np.linalg.det(np.array([[pointThree[0], 1], [pointFour[0], 1]]))
    vDenomy12 = np.linalg.det(np.array([[pointOne[1], 1], [pointTwo[1], 1]]))
    vDenomy34 = np.linalg.det(np.array([[pointThree[1], 1], [pointFour[1], 1]]))

    vXNum = np.linalg.det(np.array([[vNum12, vDenomx12], [vNum34, vDenomx34]]))
    vYNum = np.linalg.det(np.array([[vNum12, vDenomy12], [vNum34, vDenomy34]]))
    vDenom = np.linalg.det(np.array([[vDenomx12, vDenomy12], [vDenomx34, vDenomy34]]))

    result = np.array([vXNum / vDenom, vYNum / vDenom])
    #print(result)
    return result

def lineSegmentIntersectParam(pointOne, pointTwo, pointThree, pointFour):
    #pointOne and pointTwo form line segment 1, pointThree and pointFour form line segment 2
    tTop = np.array([[pointOne[0] - pointThree[0], pointThree[0] - pointFour[0]],[pointOne[1] - pointThree[1], pointThree[1] - pointFour[1]]])
    tBot = np.array([[pointOne[0] - pointTwo[0], pointThree[0] - pointFour[0]],[pointOne[1] - pointTwo[1], pointThree[1] - pointFour[1]]])
    t = np.linalg.det(tTop)/np.linalg.det(tBot)
    uTop = np.array([[pointOne[0] - pointThree[0], pointOne[0] - pointTwo[0]], [pointOne[1] - pointThree[1], pointOne[1] - pointTwo[1]]])
    uBot = np.array([[pointOne[0] - pointTwo[0], pointThree[0] - pointFour[0]], [pointOne[1] - pointTwo[1], pointThree[1] - pointFour[1]]])
    u = np.linalg.det(uTop)/np.linalg.det(uBot)
    #print(t, u)
    return np.array([t, u])

def lineSegmentIntersectBool(paramTU):
    if(0 <= paramTU[0] <= 1) and (0 <= paramTU[1] <= 1):
    	return True
    else:
        return False

def does_segment_intersect_circle(pointOne, pointTwo, circle_center, radius, roboradius):

    roboRadius = roboradius
    startToCenter = circle_center - pointOne
    endToCenter = circle_center - pointTwo
    if (getDist(startToCenter) <= radius) or (getDist(endToCenter) <= radius):
        return True

    if (pointOne[0] != pointTwo[0]) and (pointOne[1] != pointTwo[1]):
        slope = getSlope(pointOne, pointTwo)
        slopePerp = -(1 / slope) #get perp slope
        hypo = math.sqrt(1**2 + slopePerp**2)
        offset = getOffset(slopePerp, radius, hypo)
        roboRadiusOffset = getOffset(slopePerp, roboRadius, hypo)
    elif (pointOne[0] == pointTwo[0]):
        offset = np.array([radius, 0])
        roboRadiusOffset = np.array([roboRadius, 0])
    else:
        offset = np.array([0, radius])
        roboRadiusOffset = np.array([0, roboRadius])
    side = whichSide(pointOne, pointTwo, circle_center)
    if(side == 0):
        return True
    #print(side)
    pointThree = circle_center + offset
    pointFour = circle_center - offset

    v12 = pointTwo - pointOne
    pointOneOff = np.array([0, 0])
    pointTwoOff = np.array([0, 0])
    if (side > 0 and getVectorDirection(v12) > 0) or (side < 0 and getVectorDirection(v12) == 0):
        #print("Offset right")
        pointOneOff = pointOne + roboRadiusOffset
        pointTwoOff = pointTwo + roboRadiusOffset
    else:
        #print("Offset left")
        pointOneOff = pointOne - roboRadiusOffset
        pointTwoOff = pointTwo - roboRadiusOffset

    #print(pointOneOff, pointTwoOff)

    tu = lineSegmentIntersectParam(pointOneOff, pointTwoOff, pointThree, pointFour)
    TorF = lineSegmentIntersectBool(tu)
    t = tu[0]
    u = tu[1]
    return TorF

def whichSide(pointOne, pointTwo, circleCenter):
    determinant = (pointTwo[0] - pointOne[0]) * (circleCenter[1] - pointOne[1]) - (pointTwo[1] - pointOne[1]) * (circleCenter[0] - pointOne[0])
    if (determinant > 0):
        return -1
    elif(determinant < 0):
        return 1
    else:
        return 0

def getSlope(pointOne, pointTwo):
    slope = (pointTwo[1] - pointOne[1]) / (pointTwo[0] - pointOne[0])
    return slope

def getOffset(slope, radius, hypo):
    xOffset = 1 * radius / hypo
    yOffset = slope * radius / hypo
    result = np.array([xOffset, yOffset])
    return result

def getTangentPoints(pointP, pointC, radius):
    vectorCP = np.array([pointP[0] - pointC[0], pointP[1] - pointC[1]])
    vectorCPPerp = np.array([vectorCP[1] * -1, vectorCP[0]])
    vectorOP = pointP
    vectorOC = pointC
    d = getDist(vectorCP)
    vectorOT1 = vectorOC + (radius/d)**2 * vectorCP + (radius/d) * (math.sqrt(d**2-radius**2)/d) * vectorCPPerp
    vectorOT2 = vectorOC + (radius/d)**2 * vectorCP - (radius/d) * (math.sqrt(d**2-radius**2)/d) * vectorCPPerp
    result = np.array([vectorOT1, vectorOT2])
    return result

def getTangentIntersects(pointOne, pointTwo, circleCenter, radius, roboradius):
    pointOneResult = getTangentPoints(pointOne, circleCenter, radius + roboradius)
    pointTwoResult = getTangentPoints(pointTwo, circleCenter, radius + roboradius)
    tangentD1 = lineIntersectVector(pointOne, pointOneResult[0], pointTwo, pointTwoResult[1])
    tangentD2 = lineIntersectVector(pointOne, pointOneResult[1], pointTwo, pointTwoResult[0])
    #print("Tangent points")
    #print(pointOneResult)
    #print(pointTwoResult)
    #print("intersections")
    #print(tangentD1)
    #print(tangentD2)

    validity1 = validateTangentIntersection(tangentD1, listOfCircles)
    validity2 = validateTangentIntersection(tangentD2, listOfCircles)
    
    #print(validity1, validity2)
    if(validity1 == -1) and (validity2 == -1):
        print("case1")
        return tangentD1 if (getDist(tangentD1 - pointOne) <= getDist(tangentD2 - pointOne)) else tangentD2
    elif(validity1 == -1) and (validity2 != -1):
        print("case2")
        return tangentD1
    elif(validity1 != -1) and (validity2 == -1):
        print("case3")
        return tangentD2
    else:
        print("case4")
        index = validity1 if (getDist(tangentD1 - pointOne) <= getDist(tangentD2 - pointOne)) else validity2
        return getTangentIntersects(pointOne, pointTwo, listOfCircles[index,0], listOfCircles[index,1], roboradius)

def getDist(pointOne):
    return math.sqrt(pointOne[0]**2 + pointOne[1]**2)

def getVectorDirection(vector):
    if vector[1] > 0:
        return 1 #vector points upwards
    elif vector[1] < 0:
        return 0 #vector points downwards
    else:
        return 2

def validateTangentIntersection(intersect, listOfCircles):
    flag = -1
    for i in range(listOfCircles.shape[0]):
        if(getDist(listOfCircles[i,0] - intersect) <= listOfCircles[i,1]):
            flag = i
            return flag
    return flag

def getRoute(listOfLineSegmentPoints, listOfCircles, roboradius):
    flagR = False
    flagE = True
    while(flagE):
        flagE = False
        for i in range(listOfLineSegmentPoints.shape[0] - 1):
            flagR = False
            p1 = listOfLineSegmentPoints[i]
            p2 = listOfLineSegmentPoints[i + 1]
            #print(p1, p2)
            for j in range(listOfCircles.shape[0]):
                cCenter = listOfCircles[j, 0]
                cRadius = listOfCircles[j, 1]
                if(does_segment_intersect_circle(p1, p2, cCenter, cRadius, roboradius)):
                    #print(p1, p2, " intersects with ", cCenter, cRadius)
                    flagR = True
                    pMid = getTangentIntersects(p1, p2, cCenter, cRadius, roboradius + 0.2)
                    #print(pMid)
                    listOfLineSegmentPoints = np.insert(listOfLineSegmentPoints, i+1, pMid, 0)
                    #print("New List Here ---------------")
                    #print(listOfLineSegmentPoints)
                    break
            if(flagR == True):
                flagE = True
                break
    return listOfLineSegmentPoints

listResult = getRoute(listOfPoints, listOfCircles, roboradius)

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_t = 0.0
        self._max_v = 0.4
		self._max_g = 3.0

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.declare_parameter('max_v', value=self._max_v)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)


    def _listener_callback(self, msg, vel_gain=5.0, max_vel=0.2, max_pos_err=0.05):
        global index
        pose = msg.pose.pose
        max_vel = self._max_v if max_vel != self._max_v else max_vel
		vel_gain = self._max_g if vel_gain != self._max_g else vel_gain

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw
        
        maxIndex = listResult.shape[0] - 1
        self._goal_x = listResult[index, 0]
        self._goal_y = listResult[index, 1]
        
        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        t_diff = self._goal_t - cur_t
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        twist = Twist()
        if dist > max_pos_err:
            x = x_diff * vel_gain
            y = y_diff * vel_gain
            v_total = math.sqrt(x**2 + y**2)
            ratioX = x/v_total
            ratioY = y/v_total
            if v_total > max_vel:
                x = ratioX * max_vel
                y = ratioY * max_vel
            twist.linear.x = x * math.cos(t_diff) + y * math.sin(t_diff)
            twist.linear.y = -x * math.sin(t_diff) + y * math.cos(t_diff)
            self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t}) goal ({self._goal_x},{self._goal_y},{self._goal_t}) velocity ({twist.linear.x}, {twist.linear.y}) data ({x}, {y}, {x_diff}, {y_diff}, {ratioX}, {ratioY}) ({cur_t}, {t_diff})")
        else:
            if(index < maxIndex):
                index = index + 1
        
        if t_diff > max_pos_err:
            twist.angular.z = 5.0 if t_diff > 0 else -5.0
            self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t}) goal ({self._goal_x},{self._goal_y},{self._goal_t})")
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
            elif param.name == 'max_v' and param.type_ == Parameter.Type.DOUBLE:
            	self._max_v = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y} {self._goal_t}")
        return SetParametersResult(successful=True)



def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()