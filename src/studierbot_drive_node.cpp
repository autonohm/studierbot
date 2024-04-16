#include "studierbot_drive/MotorControllerCAN.h"
#include <unistd.h>
#include <cmath>


// includes for ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>


using namespace std;

// define the kinematics for the robot in a simplified version
struct Kinematic
{
    double l_x          = 0.15;         // meter
    double l_y          = 0.15;         // meter
    double wheel_radius = 0.1;          // meter
};


// define the instances of the motor controller as a global variable
constexpr unsigned int g_INSTANCES = 2; 

class StudierbotDriveNode : public rclcpp::Node
{
public:
  /**
   * Constructor to initialize the node
   * @param motorParams: MotorParams object with the motor parameters
   */
  StudierbotDriveNode(MotorParams motorParams = MotorParams())
  : Node("studierbot_drive")
  {
    _motorParams = motorParams;


    _can = std::make_unique<SocketCAN>(std::string("can0"));
    _can->startListener();

    for(unsigned int dev=0; dev<g_INSTANCES; dev++)
    {
        MotorControllerCAN* m = new MotorControllerCAN(_can.get(), dev, motorParams);
        _mc.push_back(m);
    }



    // create a subscriber to the topic cmd_vel with the type geometry_msgs::msg::Twist
    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                            "cmd_vel", 
                            10, 
                            std::bind(&StudierbotDriveNode::cmdVelCallback, this, std::placeholders::_1)
                            );


    // create a publisher for the odometry
    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);


    // send initial zero values 
    std::vector<float> rpms;
    for(unsigned int i=0 ; i<4 ; i++)
    {
        rpms.push_back(0);
    }
    this->setRPMs(rpms);


  }

  /**
   * Destructor to delete the motor controller objects
   */
  ~StudierbotDriveNode()
  {
    for(unsigned int dev=0; dev<g_INSTANCES; dev++)
    {
        delete _mc[dev];
    }
  }


private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double v_x    = msg->linear.x;
        const double v_y    = msg->linear.y;
        const double omega  = msg->angular.z;

        const double omega_1 =  (v_x + v_y - (omega * (_kinematic.l_x + _kinematic.l_y))) / _kinematic.wheel_radius * 2*M_PI;
        const double omega_2 = -(v_x - v_y + (omega * (_kinematic.l_x + _kinematic.l_y))) / _kinematic.wheel_radius * 2*M_PI;
        const double omega_3 =  (v_x - v_y - (omega * (_kinematic.l_x + _kinematic.l_y))) / _kinematic.wheel_radius * 2*M_PI;
        const double omega_4 = -(v_x + v_y + (omega * (_kinematic.l_x + _kinematic.l_y))) / _kinematic.wheel_radius * 2*M_PI;

        // output of all omega values for debugging
        RCLCPP_INFO(this->get_logger(), "Omega 1: %f, Omega 2: %f, Omega 3: %f, Omega 4: %f", omega_1, omega_2, omega_3, omega_4 );


        // create a vector of floats to store the RPM values for each motor
        std::vector<float> rpms;
        rpms.push_back(omega_1);
        rpms.push_back(omega_2);
        rpms.push_back(omega_3);
        rpms.push_back(omega_4);

        this->setRPMs(rpms);
    }




    /**
     * Function to set the RPM values for a motor controller with two channels
     * @param mc: MotorControllerCAN object
     * @param rpms: vector of two floats with the RPM values for the two channels
     */
    void setRPM(MotorControllerCAN* mc, std::vector<float> rpms)
    {
        float rpm[2];
        rpm[0] =  rpms[0];
        rpm[1] =  rpms[1];
        if(!mc->setRPM(rpm))
        {
            std::cout << "# Failed to set RPM values for CAN ID" << mc->getCanId() << std::endl;
            usleep(1000);
        }
    }




    /** 
     * Function to set the RPM values for all motor controllers 
     * @param rpms: vector of 4 floats with the RPM values for the four channels
     */
    void setRPMs(std::vector<float> rpms)
    {
        for(size_t dev = 0; dev<_mc.size(); dev++)
        {
            std::vector<float> r;
            r.push_back(rpms[2*dev]  );
            r.push_back(rpms[2*dev+1]);

            this->setRPM(_mc[dev], r);


            std::vector<float> response_vec;

            if(_mc[dev]->waitForSync())
            {
                float response[2];
                _mc[dev]->getWheelResponse(response);
                // std::cout << " " << response[0] << " " << response[1];
                response_vec.push_back(response[0] / (2*M_PI));
                response_vec.push_back(response[1] / (2*M_PI));

                

            }
            else
            {
                std::cout << "# Error synchronizing with device" << _mc[dev]->getCanId() << std::endl;
            };

            // update odometry for a mecanum drive based on the response container with omega1 to omega4
            _odom_msg.header.stamp = this->now();
            _odom_msg.header.frame_id = "odom";
            _odom_msg.child_frame_id = "base_link";

            _odom_msg.twist.twist.linear.y  = ( response_vec[0] + response_vec[1] + response_vec[2] + response_vec[3]) * _kinematic.wheel_radius / 4;
            _odom_msg.twist.twist.linear.x  = (-response_vec[0] + response_vec[1] + response_vec[2] - response_vec[3]) * _kinematic.wheel_radius / 4;
            _odom_msg.twist.twist.angular.z = (-response_vec[0] + response_vec[1] - response_vec[2] + response_vec[3]) * _kinematic.wheel_radius / (4*(_kinematic.l_x + _kinematic.l_y));

            // convert in rad


            const float dt = 1/25.0;
            _odom_msg.pose.pose.position.x      += _odom_msg.twist.twist.linear.x  * dt;
            _odom_msg.pose.pose.position.y      += _odom_msg.twist.twist.linear.y  * dt;
            _odom_msg.pose.pose.position.z       = 0.0;

            _odom_msg.pose.pose.orientation.x    = 0.0;
            _odom_msg.pose.pose.orientation.y    = 0.0;
            _odom_msg.pose.pose.orientation.z   += _odom_msg.twist.twist.angular.z * dt;
            _odom_msg.pose.pose.orientation.w    = 1.0;

            _odom_pub->publish(_odom_msg);

        }
        usleep(10000);
    }
    


    // ros varibales
    rclcpp::TimerBase::SharedPtr                               _timer;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;

    // publish the odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      _odom_pub;

    nav_msgs::msg::Odometry                                    _odom_msg;

    // variables for motor control and the kinematics   
    std::unique_ptr<SocketCAN>       _can;
    std::vector<MotorControllerCAN*> _mc;
    MotorParams                      _motorParams;
    Kinematic                        _kinematic;
};



/**
 * Main function to start the node
 * @param argc: number of arguments
 * @param argv: array of arguments
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // definition of motor parameters for the motor shields
    MotorParams motorParams;
    motorParams.frequencyScale = 32;    // PWM frequency: 1/frequencyScale x 500kHz
    motorParams.inputWeight    = 0.8f;  // Smoothing parameter for input values: smoothVal = inputWeight x prevVal + (1.f-inputWeight) x newVal
    motorParams.maxPulseWidth  = 127;   // Set maxPulse to 127 to apply full power
    motorParams.timeout        = 300;
    motorParams.gearRatio      = 59.f;
    motorParams.encoderRatio   = 1024.f;
    motorParams.rpmMax         = 100;
    motorParams.responseMode   = CAN_RESPONSE_RPM;
    motorParams.kp             = 2.f;
    motorParams.ki             = 1000.f;
    motorParams.kd             = 0.f;
    motorParams.antiWindup     = 1;
    motorParams.invertEnc      = 1;


    rclcpp::spin(std::make_shared<StudierbotDriveNode>(motorParams));
    rclcpp::shutdown();
    return 0;
}


