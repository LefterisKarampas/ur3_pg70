import rospy
from  moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.srv import ExecuteKnownTrajectory
from moveit_msgs.msg import MotionPlanResponse

joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def init():
	request = MotionPlanRequest()
	request.group_name = "manipulator";
	request.num_planning_attempts = 1;
	#request.allowed_planning_time = rospy.Duration.from_sec(5.0);

	
	cs = Constraints()
	joint_constraints = []
	goal_constraints = []


	for i in range(len(joint_names)):
		jc = JointConstraint()
		jc.joint_name = joint_names[i];
		jc.position = 0.0;
		jc.tolerance_below = 0.1;
		jc.tolerance_above = 0.1;
		joint_constraints.append(jc)


	cs.joint_constraints = joint_constraints
	goal_constraints.append(cs)
	request.goal_constraints = goal_constraints
	request.goal_constraints[0].joint_constraints[0].position = -2.0;
	request.goal_constraints[0].joint_constraints[3].position = -0.2;
	request.goal_constraints[0].joint_constraints[5].position = -0.15;
	

	rospy.wait_for_service('plan_kinematic_path')
	try:
		service = rospy.ServiceProxy('plan_kinematic_path', GetMotionPlan)
		answer = service(request)
		print 'Success:'
		print answer
		rospy.wait_for_service('execute_kinematic_path')
		service = rospy.ServiceProxy('execute_kinematic_path', ExecuteKnownTrajectory)
		answer = service(answer.motion_plan_response.trajectory, True)
	except rospy.ServiceException, e:
		print 'YOU ARE A FAILURE!'
		print e


if __name__ == '__main__':
        init()