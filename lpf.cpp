#include <ros/ros.h>
#include <std_msgs/Float64.h>

class LowPassFilter
{
public:
  LowPassFilter() : alpha_(0.5), initialized_(false), filtered_output_(0.0) {}
  LowPassFilter(double alpha) : alpha_(alpha), initialized_(false), filtered_output_(0.0) {}

  double filter(double input)
  {
    if (!initialized_)
    {
      initialized_ = true;
      filtered_output_ = input;
    }
    else
    {
      filtered_output_ = alpha_ * input + (1 - alpha_) * filtered_output_;
    }

    return filtered_output_;
  }

private:
  double alpha_;
  bool initialized_;
  double filtered_output_;
};

class FilterNode
{
public:
  FilterNode() : nh_("~")
  {
    double alpha;
    nh_.param("filter_alpha", alpha, 0.5); // 필터 상수(alpha)

    input_sub_ = nh_.subscribe("/houndx/stanley", 10, &FilterNode::inputCallback, this);
    input_sub_gps = nh_.subscribe("/houndx/stanley_gps", 10, &FilterNode::inputCallback_gps, this);
    filtered_pub_ = nh_.advertise<std_msgs::Float64>("lpf_output", 10);
    filtered_pub_gps = nh_.advertise<std_msgs::Float64>("lpf_gps_output", 10);

    filter_ = LowPassFilter(alpha);
  }

  void inputCallback(const std_msgs::Float64::ConstPtr& msg)
  {

    double filtered_value = filter_.filter(msg->data);

    std_msgs::Float64 filtered_msg;
    filtered_msg.data = filtered_value;


    filtered_pub_.publish(filtered_msg);
  }
  void inputCallback_gps(const std_msgs::Float64::ConstPtr& msg)
  {

    double filtered_value_gps = filter_.filter(msg->data);


    std_msgs::Float64 filtered_msg_gps;
    filtered_msg_gps.data = filtered_value_gps;


    filtered_pub_gps.publish(filtered_msg_gps);
  }
private:
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;
  
  ros::Subscriber input_sub_gps;
  ros::Publisher filtered_pub_;
  ros::Publisher filtered_pub_gps;
  LowPassFilter filter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_node");

  FilterNode filter_node;

  ros::spin();

  return 0;
}