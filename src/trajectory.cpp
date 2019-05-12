#include <ros/ros.h>
#include <ur_kinematics/ur_kin.h>
//#include "assignment1.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>
// #include<rob_param.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <trajectory_msgs/JointTrajectory.h>
#define PI 3.14159
class Trajectory {

private:

  ros::NodeHandle nh;
  ros::Subscriber sub_setpoint;
  ros::Subscriber sub_jointstate;
  ros::Publisher  pub_trajectory;

  double shoulder_pan_joint;
  double shoulder_lift_joint;
  double elbow_joint;
  double wrist_1_joint;
  double wrist_2_joint;
  double wrist_3_joint;

  tf::TransformListener listener;
  // typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
  // Client action_client(
  //     "arm_controller/follow_joint_trajectory", // For simulation
  //     true);
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* action_client;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};

  
  // This reads the first three joints
  void JointState( const sensor_msgs::JointState& jointstate ){ 

    for( size_t i=0; i<jointstate.name.size(); i++ ){
       // std::cout<<"jointstatae name: "<<i<<jointstate.name[i]<<std::endl;
      if( jointstate.name[i] == "shoulder_pan_joint" )
	   { shoulder_pan_joint = jointstate.position[i]; }
      if( jointstate.name[i] == "shoulder_lift_joint" )
	   { shoulder_lift_joint = jointstate.position[i]; }
      if( jointstate.name[i] == "elbow_joint" )
	   { elbow_joint = jointstate.position[i]; }
      if( jointstate.name[i]=="wrist_1_joint")
        { wrist_1_joint=jointstate.position[i];}
      if( jointstate.name[i]=="wrist_2_joint")
        { wrist_2_joint=jointstate.position[i];}
      if( jointstate.name[i]=="wrist_3_joint")
      {wrist_3_joint=jointstate.position[i];}
    }
  }

  // Main callback used when a new setpoint is received
  void SetPoint( const geometry_msgs::Pose& goaltransform){

    double positionincrement = 1e-5; // how much we move between steps

    // The name of all the joints
    control_msgs::FollowJointTrajectoryGoal controlTrajectory;

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back( "shoulder_pan_joint" );
    trajectory.joint_names.push_back( "shoulder_lift_joint" );
    trajectory.joint_names.push_back( "elbow_joint" );

    trajectory.joint_names.push_back( "wrist_1_joint" );
    trajectory.joint_names.push_back( "wrist_2_joint" );
    trajectory.joint_names.push_back( "wrist_3_joint" );

    // The trajectory point. Initialized with current values
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.push_back( shoulder_pan_joint );
    trajectory_point.positions.push_back( shoulder_lift_joint );
    trajectory_point.positions.push_back( elbow_joint );

    trajectory_point.positions.push_back( wrist_1_joint);
    trajectory_point.positions.push_back( wrist_2_joint- 0.0 );
    trajectory_point.positions.push_back( wrist_3_joint- 0.0 );
    trajectory_point.time_from_start = ros::Duration( 0.0 );

    trajectory_point.velocities=zero_vector;
    trajectory.points.push_back(trajectory_point);


    tf::Quaternion quatgoal(goaltransform.orientation.x, goaltransform.orientation.y,goaltransform.orientation.z, goaltransform.orientation.w);
    
    Eigen::Vector3d posgoal(goaltransform.position.x, goaltransform.position.y, goaltransform.position.z);



    Eigen::Matrix<double,3,3> R;
    R=quat2rotm(quatgoal);
    Eigen::Matrix<double,4, 4> goalFrame=Eigen::Matrix4d::Identity();
    double* T =new double[16];
    goalFrame.block(0,0,3,3)<<R;
    
    goalFrame.block(0,3,3,1)<<posgoal;
    // std::cout<<goalFrame<<std::endl;
    // std::cout<<goalFrame(0,3)<<std::endl;
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
      {
        // std::cout<<"goalframe value: "<<goalFrame(i,j);
        *T=goalFrame(i,j);
        T++;
      }
    }
    
    Eigen::VectorXf q_curr(6);
    for(int i=0;i<6;i++)
      q_curr(i)=(trajectory_point.positions[i]);
    
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
      {
        
        T--;
      }
    }

    double* q_sols=new double[8*6];
    int numOfSol=0;
    numOfSol=ur_kinematics::inverse(T, q_sols, 0);
    // std::cout<<numOfSol<<" after inverse"<<std::endl;
    //TODO get correct solutions for q_sols

    Eigen::Matrix<double, 8,6> all_q_possible;
    Eigen::VectorXf q_goal(6);
    

    double dist=100000;
    for(int i=0;i<numOfSol;i++)
    {
      for(int j=0;j<6;j++)
      {
        all_q_possible(i,j)=*q_sols;
        if(all_q_possible(i,j)>M_PI)
        {
          all_q_possible(i,j)-=2*3.14159;
        }
        q_sols++;
      }
      // dist=(all_q_possible(-q_curr)
      Eigen::VectorXf soli(6);
      soli(0)=all_q_possible(i,0);
      soli(1)=all_q_possible(i,1);
      soli(2)=all_q_possible(i,2);
      soli(3)=all_q_possible(i,3);
      soli(4)=all_q_possible(i,4);
      soli(5)=all_q_possible(i,5);
      // std::cout<<"solution i: "<<soli<<std::endl;
      // soli=all_q_possible.col(i);
      if((soli-q_curr).norm()<dist)
        {

          dist=(soli-q_curr).norm();
          // std::cout<<"dist "<<dist<<std::endl;
          q_goal=soli;
        }
    }
  

    // std::cout<<"goal "<<q_goal<<std::endl;
    // std::cout<<"curr: "<<q_curr<<std::endl;
    ROS_INFO_STREAM("goal");
    ROS_INFO_STREAM(q_goal);
    ROS_INFO_STREAM("curr");
    ROS_INFO_STREAM(q_curr);

    // This is the translation left for the trajectory
    
      // trajectory.points.push_back( trajectory_point );

    Eigen::VectorXf translation(6);
    translation=q_goal-q_curr;
    // std::cout<<"Translation: "<<translation<<std::endl;

    int subSteps = 100;
    int count=1;
    // trajectory_msgs::JointTrajectoryPoint final_point;
    // final_point.velocities=zero_vector;

    for(int i=0;i<6;i++)
    {
      trajectory_point.positions[i]=q_goal(i);
    }
    // std::cout<<"here"<<std::endl;
    trajectory_point.time_from_start=ros::Duration(8);
    trajectory.points.push_back(trajectory_point);
    // std::cout<<"here 2"<<std::endl;

    // while((positionincrement)<translation.norm()&&numOfSol!=0)
    // {
    //   trajectory.points.push_back( trajectory_point );
    //   q_curr=q_curr+translation/translation.norm()*positionincrement;
    //   for(int i=0;i<6;i++)
    //   {
    //     trajectory_point.positions[i]=q_curr(i);
    //   }

    //   trajectory_point.time_from_start += ros::Duration( positionincrement*10 );
      
    //  // std::cout<<q_curr<<std::endl<<std::endl;
    //   translation=q_goal-q_curr;



    // }
    // std::cout<<trajectory<<std::endl;
    // pub_trajectory.publish(trajectory);

    controlTrajectory.trajectory = trajectory;
    // std::cout<<"before send"<<std::endl;
    action_client->sendGoalAndWait(controlTrajectory);
    // action_client->waitForResult();
    // ros::Duration(2).sleep();
    // std::cout<<"after duration"<<std::endl;
    ROS_INFO_STREAM("after duration");
    // delete q_sols;
    // delete T;

 
  }

public:

  Trajectory( ros::NodeHandle& nh ):
    nh( nh ){

      
    sub_setpoint=nh.subscribe( "setpoint",1,&Trajectory::SetPoint,this );
    sub_jointstate=nh.subscribe( "joint_states",1,&Trajectory::JointState,this );
    // pub_trajectory=nh.advertise<trajectory_msgs::JointTrajectory>( "/arm_controller/follow_joint_trajectory", 1 );
    // action_client=new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>( "/arm_controller/follow_joint_trajectory", // For simulation
    //   true);
     action_client=new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>( "/follow_joint_trajectory", // For simulation
      true);

      ROS_INFO("Waiting for action server to start");
  bool server_exists = action_client->waitForServer(ros::Duration(3.0));
  while (!server_exists) // Will wait forever
  {
    ROS_WARN("Still waiting...");
    ros::spinOnce();
    ros::Duration(1.0).sleep(); // Retry after 1 second
    server_exists = action_client->waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("Action server connected!");
  }

private:





  Eigen::Matrix<double,3,3> quat2rotm(tf::Quaternion q)
  {
    float qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z();
    // std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
      Eigen::Matrix<double,3,3> R;
      R(0,0)=1-2*qy*qy-2*qz*qz;
      R(0,1)=2*qx*qy-2*qz*qw;
      R(0,2)=2*qx*qz+2*qy*qw;
      R(1,0)=2*qx*qy+2*qz*qw;
      R(1,1)=1-2*qx*qx-2*qz*qz;
      R(1,2)=2*qy*qz-2*qx*qw;
      R(2,0)=2*qx*qz-2*qy*qw;
      R(2,1)=2*qy*qz+2*qx*qw;
      R(2,2)=1-2*qx*qx-2*qy*qy;
      return R;
  }




};



int main( int argc, char** argv ){

  // This must be called for every node
  ros::init( argc, argv, "trajectory" );

  // Create a node handle
  ros::NodeHandle nh;

  Trajectory trajectory( nh );

  ros::spin();

  return 0;

}




// #include <ros/ros.h>
// #include <ur_kinematics/ur_kin.h>
// //#include "assignment1.hpp"
// #include <Eigen/Eigen>
// #include <Eigen/Dense>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
// #include <sensor_msgs/JointState.h>
// #include <geometry_msgs/Point.h>
// #include<geometry_msgs/Pose.h>
// #include <geometry_msgs/Pose.h>
// // #include<rob_param.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include<control_msgs/FollowJointTrajectoryAction.h>
// #include <pr2_controllers_msgs/JointTrajectoryAction.h>

// #include <trajectory_msgs/JointTrajectory.h>
// #define PI 3.14159
// class Trajectory {

// private:

//   ros::NodeHandle nh;
//   ros::Subscriber sub_setpoint;
//   ros::Subscriber sub_jointstate;
//   ros::Publisher  pub_trajectory;

//   double shoulder_pan_joint;
//   double shoulder_lift_joint;
//   double elbow_joint;
//   double wrist_1_joint;
//   double wrist_2_joint;
//   double wrist_3_joint;

//   tf::TransformListener listener;
//   // typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> Client;

//   //TODO shift to controller_msgs::followaction. 
//   //TODO figure out action client
//   // This reads the first three joints
//   void JointState( const sensor_msgs::JointState& jointstate ){ 

//     for( size_t i=0; i<jointstate.name.size(); i++ ){
//        // std::cout<<"jointstatae name: "<<i<<jointstate.name[i]<<std::endl;
//       if( jointstate.name[i] == "shoulder_pan_joint" )
//      { shoulder_pan_joint = jointstate.position[i]; }
//       if( jointstate.name[i] == "shoulder_lift_joint" )
//      { shoulder_lift_joint = jointstate.position[i]; }
//       if( jointstate.name[i] == "elbow_joint" )
//      { elbow_joint = jointstate.position[i]; }
//       if( jointstate.name[i]=="wrist_1_joint")
//         { wrist_1_joint=jointstate.position[i];}
//       if( jointstate.name[i]=="wrist_2_joint")
//         { wrist_2_joint=jointstate.position[i];}
//       if( jointstate.name[i]=="wrist_3_joint")
//       {wrist_3_joint=jointstate.position[i];}
//     }
//   }

//   // Main callback used when a new setpoint is received
//   void SetPoint( const geometry_msgs::Pose& goaltransform){

//     double positionincrement = 1e-3; // how much we move between steps

//     // The name of all the joints
//     trajectory_msgs::JointTrajectory trajectory;
//     trajectory.joint_names.push_back( "shoulder_pan_joint" );
//     trajectory.joint_names.push_back( "shoulder_lift_joint" );
//     trajectory.joint_names.push_back( "elbow_joint" );

//     trajectory.joint_names.push_back( "wrist_1_joint" );
//     trajectory.joint_names.push_back( "wrist_2_joint" );
//     trajectory.joint_names.push_back( "wrist_3_joint" );

//     // The trajectory point. Initialized with current values
//     trajectory_msgs::JointTrajectoryPoint trajectory_point;
//     trajectory_point.positions.push_back( shoulder_pan_joint );
//     trajectory_point.positions.push_back( shoulder_lift_joint );
//     trajectory_point.positions.push_back( elbow_joint );

//     trajectory_point.positions.push_back( wrist_1_joint);
//     trajectory_point.positions.push_back( wrist_2_joint- 0.0 );
//     trajectory_point.positions.push_back( wrist_3_joint- 0.0 );
//     trajectory_point.time_from_start = ros::Duration( positionincrement );

    

//     tf::Quaternion quatgoal(goaltransform.orientation.x, goaltransform.orientation.y,goaltransform.orientation.z, goaltransform.orientation.w);
    
//     Eigen::Vector3d posgoal(goaltransform.position.x, goaltransform.position.y, goaltransform.position.z);



//     Eigen::Matrix<double,3,3> R;
//     R=quat2rotm(quatgoal);
//     Eigen::Matrix<double,4, 4> goalFrame=Eigen::Matrix4d::Identity();
//     double* T =new double[16];
//     goalFrame.block(0,0,3,3)<<R;
    
//     goalFrame.block(0,3,3,1)<<posgoal;
//     std::cout<<goalFrame<<std::endl;
//     std::cout<<goalFrame(0,3)<<std::endl;
//     for(int i=0;i<4;i++)
//     {
//       for(int j=0;j<4;j++)
//       {
//         // std::cout<<"goalframe value: "<<goalFrame(i,j);
//         *T=goalFrame(i,j);
//         T++;
//       }
//     }
    
//     Eigen::VectorXf q_curr(6);
//     for(int i=0;i<6;i++)
//       q_curr(i)=(trajectory_point.positions[i]);
    
//     for(int i=0;i<4;i++)
//     {
//       for(int j=0;j<4;j++)
//       {
        
//         T--;
//       }
//     }

//     double* q_sols=new double[8*6];
//     int numOfSol=0;
//     numOfSol=ur_kinematics::inverse(T, q_sols, 0);
//     std::cout<<numOfSol<<" after inverse"<<std::endl;
//     //TODO get correct solutions for q_sols

//     Eigen::Matrix<double, 8,6> all_q_possible;
//     Eigen::VectorXf q_goal(6);
    

//     double dist=100000;
//     for(int i=0;i<numOfSol;i++)
//     {
//       for(int j=0;j<6;j++)
//       {
//         all_q_possible(i,j)=*q_sols;
//         if(all_q_possible(i,j)>M_PI)
//         {
//           all_q_possible(i,j)-=2*3.14159;
//         }
//         q_sols++;
//       }
//       // dist=(all_q_possible(-q_curr)
//       Eigen::VectorXf soli(6);
//       soli(0)=all_q_possible(i,0);
//       soli(1)=all_q_possible(i,1);
//       soli(2)=all_q_possible(i,2);
//       soli(3)=all_q_possible(i,3);
//       soli(4)=all_q_possible(i,4);
//       soli(5)=all_q_possible(i,5);
//       std::cout<<"solution i: "<<soli<<std::endl;
//       // soli=all_q_possible.col(i);
//       if((soli-q_curr).norm()<dist)
//         {

//           dist=(soli-q_curr).norm();
//           std::cout<<"dist "<<dist<<std::endl;
//           q_goal=soli;
//         }
//     }
  

//     std::cout<<"goal "<<q_goal<<std::endl;
//     std::cout<<"curr: "<<q_curr<<std::endl;


//     // This is the translation left for the trajectory
    

//     Eigen::VectorXf translation(6);
//     translation=q_goal-q_curr;
//     std::cout<<"Translation: "<<translation<<std::endl;

//     int subSteps = 100;
//     int count=1;
//     while(positionincrement<translation.norm()&&numOfSol!=0)
//     {
      
//       for(int i=0;i<6;i++)
//       {
//         trajectory_point.positions[i]=q_curr(i);
//       }

//       trajectory_point.time_from_start += ros::Duration( positionincrement*10 );
//       trajectory.points.push_back( trajectory_point );
//       q_curr=q_curr+translation/translation.norm()*positionincrement;
//      // std::cout<<q_curr<<std::endl<<std::endl;
//       translation=q_goal-q_curr;



//     }
//     // std::cout<<trajectory<<std::endl;
//     pub_trajectory.publish(trajectory);
//     ros::Duration(2).sleep();
//     std::cout<<"after duration"<<std::endl;
//     // delete q_sols;
//     // delete T;

 
//   }

// public:

//   Trajectory( ros::NodeHandle& nh ):
//     nh( nh ){

//       // Client action_client(
//       // "arm_controller/command", // For simulation
//       // true);
//     sub_setpoint=nh.subscribe( "setpoint",1,&Trajectory::SetPoint,this );
//     sub_jointstate=nh.subscribe( "joint_states",1,&Trajectory::JointState,this );
//     pub_trajectory=nh.advertise<trajectory_msgs::JointTrajectory>( "/arm_controller/command", 1 );
//   }

// private:





//   Eigen::Matrix<double,3,3> quat2rotm(tf::Quaternion q)
//   {
//     float qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z();
//     std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
//       Eigen::Matrix<double,3,3> R;
//       R(0,0)=1-2*qy*qy-2*qz*qz;
//       R(0,1)=2*qx*qy-2*qz*qw;
//       R(0,2)=2*qx*qz+2*qy*qw;
//       R(1,0)=2*qx*qy+2*qz*qw;
//       R(1,1)=1-2*qx*qx-2*qz*qz;
//       R(1,2)=2*qy*qz-2*qx*qw;
//       R(2,0)=2*qx*qz-2*qy*qw;
//       R(2,1)=2*qy*qz+2*qx*qw;
//       R(2,2)=1-2*qx*qx-2*qy*qy;
//       return R;
//   }




// };



// int main( int argc, char** argv ){

//   // This must be called for every node
//   ros::init( argc, argv, "trajectory" );

//   // Create a node handle
//   ros::NodeHandle nh;

//   Trajectory trajectory( nh );

//   ros::spin();

//   return 0;

// }






