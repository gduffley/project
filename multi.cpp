#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
//test

class multi{
public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topic
	multi(ros::NodeHandle& nh) :
		fsm(FSM_MOVE_FORWARD),
		msf(MSF_MOVE_FORWARD), 
		rotateStartTime(ros::Time::now()),
		rotateDuration(0.f) {
		// Initialize random time generator
		srand(time(NULL));
		
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		commandPub0 = nh.advertise < geometry_msgs::Twist > ("robot_0/cmd_vel", 1);
		commandPub1 = nh.advertise < geometry_msgs::Twist > ("robot_1/cmd_vel", 1);
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		laserSub0 = nh.subscribe("robot_0/base_scan", 1, &multi::commandCallback0,this);
		laserSub1 = nh.subscribe("robot_1/base_scan", 1, &multi::commandCallback0,this);
	};
 
// Send a velocity command
void dummy(int a){};
void move0(double linearVelMPS, double angularVelRadPS) {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = linearVelMPS;
	msg.angular.z = angularVelRadPS;
	commandPub0.publish(msg);
};

void move1(double linearVelMPS, double angularVelRadPS) {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = linearVelMPS;
	msg.angular.z = angularVelRadPS;
	commandPub1.publish(msg);
};
	

// Process the incoming laser scan message
void commandCallback0(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if (fsm == FSM_MOVE_FORWARD) {
		//Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
		//Ideally the loops should check for boundary issues by computing
		// -currAngle = msg -> angle_min + msg -> angle_increment*currIndex
		//ensuring that currAngle <= msg -> angle_max
unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
float closestRange = msg->ranges[minIndex];
	for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
		if (msg->ranges[currIndex] < closestRange) {
			closestRange = msg->ranges[currIndex];
		}
	}
	ROS_INFO_STREAM("Range0: " << closestRange);
	//TODO if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime

//*****************************************************ANSWER CODE BEGIN
		if (PROXIMITY_RANGE_M >= closestRange){
			fsm = FSM_ROTATE;			
			rotateStartTime = ros::Time::now();
			//Duration 1-3 sec, rotation of 90-270
			//Duration 0-4 sec, rotation of 0-360				
			rotateDuration=ros::Duration (rand() % (int)((2*M_PI)/ROTATE_SPEED_RADPS));
		}
//*****************************************************ANSWER CODE END
		}
	};

// Process the incoming laser scan message
void commandCallback1(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if (msf == MSF_MOVE_FORWARD) {
		//Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
		//Ideally the loops should check for boundary issues by computing
		// -currAngle = msg -> angle_min + msg -> angle_increment*currIndex
		//ensuring that currAngle <= msg -> angle_max
unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
float closestRange = msg->ranges[minIndex];
	for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
		if (msg->ranges[currIndex] < closestRange) {
			closestRange = msg->ranges[currIndex];
		}
	}
	ROS_INFO_STREAM("Range1: " << closestRange);
	//TODO if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime

//*****************************************************ANSWER CODE BEGIN
		if (PROXIMITY_RANGE_M >= closestRange){
			msf = MSF_ROTATE;			
			rotateStartTime = ros::Time::now();
			//Duration 1-3 sec, rotation of 90-270
			//Duration 0-4 sec, rotation of 0-360				
			rotateDuration=ros::Duration (rand() % (int)((2*M_PI)/ROTATE_SPEED_RADPS));
		}
//*****************************************************ANSWER CODE END
		}
	};

	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(50); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			// TODO: Either call:
			//       move(0, ROTATE_SPEED_RADPS); // Rotate right
			//       or
			//       move(FORWARD_SPEED_MPS, 0); // Move foward
			//       depending on FSM state; change the FSM state when appropriate

//*****************************************************ANSWER CODE BEGIN
			if (fsm == FSM_MOVE_FORWARD) move0(FORWARD_SPEED_MPS, 0.0);
			else {
				move0(0.0, ROTATE_SPEED_RADPS);
				if (ros::Time::now() >= rotateStartTime + rotateDuration) {
					fsm = FSM_MOVE_FORWARD;
				}
			}
			if (msf = MSF_MOVE_FORWARD) move1(FORWARD_SPEED_MPS, 0.0);
			else {
				move1(0.0, ROTATE_SPEED_RADPS);
				if (ros::Time::now() >= rotateStartTime + rotateDuration) {
					msf = MSF_MOVE_FORWARD;
				}
			}
//*****************************************************ANSWER CODE END
			
		ros::spinOnce(); //Call this function often to process incoming
		rate.sleep(); // Sleep the rest of the cycle, for FSM loop rate
		}
	};

	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
	enum MSF {MSF_MOVE_FORWARD, MSF_ROTATE};
	
	// Tunable parameters
	// TODO: tune parameters as you see fit
	const static double MIN_SCAN_ANGLE_RAD = -10.0 / 180 * M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +10.0 / 180 * M_PI;
	const static float PROXIMITY_RANGE_M = 1.5;
	const static double FORWARD_SPEED_MPS = 50.0;
	const static double ROTATE_SPEED_RADPS = M_PI / 2;

protected:
	ros::Publisher commandPub0; // Publisher to robot's velocity command topic
	ros::Subscriber laserSub0; // Subscriber to the simulated robot's laser scan topic
	ros::Publisher commandPub1;
	ros::Subscriber laserSub1;
	enum FSM fsm; // Finite state machine for the first random walker
	enum MSF msf; // Machine state finite for the second random walker
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "multi"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	multi dots(n); // Create new random walk object
	dots.spin(); // Execute FSM loop
	
	return 0;
}	
