#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
	for(int i=0;i<20;i++)
	{
		std::cout<<i<<std::endl;
	}
	std::cout<<"in cpp file before"<<std::endl;
	ros::init(argc, argv, "camera_listener");
	ros::NodeHandle nh;
	ros::Duration(10.0).sleep();
	tf::TransformListener l;
	std::cout<<"in cpp file"<<std::endl;
	ros::Rate rate(10.0);
	while(nh.ok())
	{
		std::cout<<"in cpp file"<<std::endl;
		tf::StampedTransform t;
	try {
		l.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
		l.lookupTransform("/base_link", "/ee_link", ros::Time(0), t);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("Not found");
		ros::Duration(1.0).sleep();
		continue;
	}

	std::cout<<t.getOrigin().x()<<std::endl;
	std::cout<<t.getOrigin().y()<<std::endl;	
	std::cout<<t.getOrigin().z()<<std::endl;
	std::cout<<t.getRotation().x()<<std::endl;
	std::cout<<t.getRotation().y()<<std::endl;
	std::cout<<t.getRotation().z()<<std::endl;
	std::cout<<t.getRotation().w()<<std::endl;
	rate.sleep();
	}
	return 0;
};
