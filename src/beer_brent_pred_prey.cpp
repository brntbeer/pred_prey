#include <ros/ros.h>
#include "time.h"
#include "joy/Joy.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "cmvision/Blobs.h"
#include "cmvision/Blob.h"
#include "wiimote/State.h"
#include "wiimote/RumbleControl.h"
#include "geometry_msgs/Twist.h"

//Wiimote contols
#define BUTTON_1     0
#define BUTTON_2     1
#define BUTTON_A     2
#define BUTTON_B     3
#define BUTTON_PLUS  4
#define BUTTON_MINUS 5
#define BUTTON_LEFT  6
#define BUTTON_RIGHT 7
#define BUTTON_DOWN  8
#define BUTTON_UP    9
#define BUTTON_HOME  10

//color models
#define ORANGE 324
#define RED    255
#define GREEN  255
#define BLUE   255

//Behavior Types
#define HOG    0
#define ROAM   1
#define HUNGRY 2
#define EAT    3
#define FLEE   4
#define HOME   5
#define AVOID  6


using namespace std;
double linear_vel = 0.3;
double angular_vel = 0.3;
int behavior = HOG; //used to set a behavior to work with
int eat_counter = 0;
ros::Publisher movementPub;
ros::Publisher rumblePub; //used for bonus points on p1b
ros::Publisher feedingPub;
std_msgs::Bool feeding;
wiimote::RumbleControl rmbl;

void wiimote_callback(const wiimote::StateConstPtr& msg);
void health_control(const std_msgs::Int8ConstPtr& msg);
void vision_processing(const cmvision::BlobsConstPtr& msg);
void laser_scan(const sensor_msgs::LaserScanConstPtr& msg);
void arbiter();
void flee();
void roam();
void eat();
void turn(double angular);
void drive_straight(double linear);
bool line_up(int left, int right);
void look_for_food();
void look_for_home();

int main(int argc, char **argv)
{

	ros::init (argc, argv, "beer_brent_hunger_node");
	ros::NodeHandle n;
	ros::Subscriber simpleSub;
	ros::Subscriber hungerSub;
	ros::Subscriber imageSub;
	ros::Subscriber laserSub;


	simpleSub = n.subscribe("wiimote/state",1, wiimote_callback);
	hungerSub = n.subscribe("/health",1,health_control);
	imageSub  = n.subscribe("/blobs",1,vision_processing);
	laserSub  = n.subscribe("/scan",1,laser_scan);

	feedingPub = n.advertise<std_msgs::Bool>("/feeding", 1);
	rumblePub = n.advertise<wiimote::RumbleControl>("wiimote/rumble", 1);
	movementPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(5);
	while(ros::ok())
	{
		ros::spinOnce();
		arbiter();

		ROS_INFO("Behavior: %d", behavior);
		loop_rate.sleep();
	}
	return 0;
}

void wiimote_callback(const wiimote::StateConstPtr& msg)
{
	//check on all buttons and perform their actions
	if(msg->buttons[BUTTON_2])
	{ 
		//deadman switch to start the robot
		behavior = HOG;
		if(msg->buttons[BUTTON_UP])
		{ 
			drive_straight(linear_vel);
			//rmbl.rumble.switch_mode = 1;
		}
		if(msg->buttons[BUTTON_DOWN])
		{
			drive_straight(-linear_vel);
			//rmbl.rumble.switch_mode = 0;
		}
		if(msg->buttons[BUTTON_LEFT])
		{
			turn(angular_vel);
		}
		if(msg->buttons[BUTTON_RIGHT])
		{
			turn(-angular_vel);
		}
		//rumblePub.publish(rmbl);
	}
	if(msg->buttons[BUTTON_1])
	{
		behavior = ROAM;
	}

}

void health_control(const std_msgs::Int8ConstPtr& msg)
{
	//ROS_INFO("Health Control Called");
	if(msg->data <= 10 && behavior != HOG)
	{
		//	ROS_INFO("I am Hungry! Looking for food");
		ROS_INFO("Hunger Level: %d", msg->data);
		behavior = HUNGRY;
	}
	else if (msg->data >= 50)
	{
		if(behavior== EAT)
		{
			eat_counter++;
			ROS_INFO("JUST ATE: %d", eat_counter);
		}
		behavior = ROAM;
		feeding.data = false;
		ROS_INFO("Hunger Level: %d", msg->data );
		feedingPub.publish(feeding);
	}
	else
	{
		feeding.data = false;
		ROS_INFO("Hunger Level: %d", msg->data );
	}
}

void vision_processing(const cmvision::BlobsConstPtr& msg)
{
	bool friend_seen = false;
	for(int i=0; i < msg->blob_count; i++)
	{
		//dont know what would happen to movement if i removed
		//HOG check, would color tracking still report but not move?
		if(msg->blobs[i].area > 300 && behavior != HOG)
		{
			//check to see what color it is, and act accordingly

			if((msg->blobs[i].red + msg->blobs[i].green) == ORANGE 
					&& behavior == HOME )
			{
				ROS_INFO("Home detected at (%d,%d).", msg->blobs[i].x, msg->blobs[i].y);
				//while it's not line up and still sees home
				if(!line_up(msg->blobs[i].left, msg->blobs[i].right)
						&& (msg->blobs[i].red+msg->blobs[i].green) == ORANGE)
				{
					if(msg->blobs[i].area > 20000)
					{
						ROS_INFO("I am home");
					}
				}
			}
			if(msg->blobs[i].red == RED && msg->blobs[i].green < 1 
					&& friend_seen == false)
			{
				//ROS_INFO("Red blob detected! area: %d, location: (%d,%d)", msg->blobs[i].area, 
				//	msg->blobs[i].x, msg->blobs[i].y);
				behavior = FLEE;
			}
			if(msg->blobs[i].green == GREEN && behavior == HUNGRY)
			{
				ROS_INFO("Green blob detected! area: %d,", msg->blobs[i].area); 
				// while it's not lined up, or no longer seeing green
				if(line_up(msg->blobs[i].left, msg->blobs[i].right)
						&& msg->blobs[i].green == GREEN)
				{

					ROS_INFO("lined up");
					//blobs all up in this bitch.
					if(msg->blobs[i].area > 25000 || msg->blobs[i].top > 400)
					{
						behavior = EAT;
						ROS_INFO("Eating");
					}
					//may need some behavior to move forward a little 
					//to ensure you're on the blob
					//or instead of area > 25000, check to see if the 
					//top of the bounding box is within a portion of the
					//screen
				}
			}
		}
		if(msg->blobs[i].blue == BLUE
				&& msg->blobs[i].area > 500)
		{
			//ROS_INFO("Blue blob detected! area: %d, location: (%d,%d)", msg->blobs[i].area, 
			//		msg->blobs[0].x, msg->blobs[i].y);
			ROS_INFO("HI FRIEND");
			friend_seen = true;

		}
	}
}
}
void laser_scan(const sensor_msgs::LaserScanConstPtr& msg)
{
	//loop through the data points.
	//find highest value? (between ~160-360)
	//or if the first value found over 1.5 is at index >280 or so, turn left.
	//else turn right
	for(int i=0; i < 512; i++)
	{
		// ROS_INFO("laser, index: %f,%d", msg->ranges[i],i);
		if(i/3 > 60 && i/3 < 120 )
		{
			if(msg->ranges[i] <= 0.8 && msg->ranges[i] > msg->range_min)
			{
				if(i/3 > 90)
				{
					//turn left
				  ROS_INFO("Turning Left");
					turn(angular_vel);
				}
				if(i/3 >= 90)
				{
					//turn right
					ROS_INFO("Turning Right");
					turn(-angular_vel);
				}
			}
			//call obs avoidance.
		}
	}

}

void arbiter()
{
	if(behavior == AVOID)
	{
		obstacle_avoidance();
	}
	if(behavior == HOME)
	{
		go_home();
	}
	if(behavior == FLEE)
	{
		//run the fck away
		flee();
	}
	//sit there and eat and publish that you're eating
	if(behavior == EAT)
	{
		eat();
	}
	if(behavior == HUNGRY)
	{
		look_for_food();
	}
	//continue roaming
	if(behavior == ROAM)
	{
		roam();
	}
}

//move around in a semi-random function
//also avoid obstacles and predators
void roam()
{
	int num;
	//array of forward left right commands to randomly choose 
	//which to do and then move forward for a fixed distance.
	srand((unsigned)time(NULL) );
	num = (1+rand() % 6);
	if(num == 1) //forward
	{
		drive_straight(linear_vel);
	}
	if(num == 2 || num == 4) //left
	{
		turn(angular_vel);
		//drive_straight(linear_vel);
	}
	if(num == 3 || num == 5) //right
	{
		turn(-angular_vel);
		//drive_straight(linear_vel);
	}
}


//once a patch of food is found go eat it
//eat until hunger is high enough.
//this function will also publish the hunger information back
void eat()
{
	//move forward until the blob is of sufficient size
	// (~25,000 for area)
	//
	//also stop the movement
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	feeding.data = true;
	feedingPub.publish(feeding);
	movementPub.publish(cmd_vel);
	if(eat_counter >=3)
	{
		behavior = HOME;
	}
}

//run away from a predator as you see it.
//most probably line yourself up with it
//then drive backwards to keep an eye on it
//try not to get stuck staring at the predator too long
void flee()
{
	ROS_INFO("FLEEEEEEEEEEEEEE\n");
	drive_straight(-linear_vel);
	turn(-angular_vel+0.2);
	behavior = ROAM;
}
void turn(double angular)
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.angular.z = angular;
	movementPub.publish(cmd_vel);
}
void drive_straight(double linear)
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	movementPub.publish(cmd);
}
bool line_up(int left, int right)
{
	if(left < 213 && right < 213)
	{
		//turn left
		ROS_INFO("Turning left");
		turn(angular_vel+angular_vel);
		return false;
	}
	else if(left < 213 && right > 213	&& right < 427)
	{
		ROS_INFO("Turning Left little");
		turn(angular_vel*angular_vel);
		return false;
	}
	else if(left > 427 && right > 427)
	{
		//turn right
		ROS_INFO("Turning Right");
		turn(-angular_vel-angular_vel);
		return false;
	}
	else if(left > 213 && left < 427 && right > 427)
	{
		ROS_INFO("Turning Right Little");
		turn(-angular_vel*(angular_vel));
		return false;
	}
	else if(left > 213 && right < 427)
	{
		//move forward a lot
		ROS_INFO("Moving Forward");
		drive_straight(linear_vel);
		return false;
	}
	else
		return true;
}
void look_for_food()
{
	turn(angular_vel);
	drive_straight(linear_vel);
	//cmd.angular.z = angular_vel;
	//movementPub.publish(cmd);
}
void look_for_home()
{

	turn(angular_vel);
	drive_straight(linear_vel);
}
