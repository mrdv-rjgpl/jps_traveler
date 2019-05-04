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

  // This reads the first three joints
  void JointState( const sensor_msgs::JointState& jointstate ){ 
    //TODO add jointstate names

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
  void SetPoint( const geometry_msgs::Pose& goaltransform){//const tfge::Transform& goaltransform){//geometry_msgs::Point& newgoal ){ //geometrymsgs::pose

    double positionincrement = 1e-2; // how much we move between steps

    // The name of all the joints
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back( "shoulder_pan_joint" );
    trajectory.joint_names.push_back( "shoulder_lift_joint" );
    trajectory.joint_names.push_back( "elbow_joint" );
    // trajectory.joint_names.push_back( "elbow_joint" );
    // trajectory.joint_names.push_back( "shoulder_lift_joint" );
    // trajectory.joint_names.push_back( "shoulder_pan_joint" );


    trajectory.joint_names.push_back( "wrist_1_joint" );
    trajectory.joint_names.push_back( "wrist_2_joint" );
    trajectory.joint_names.push_back( "wrist_3_joint" );

    // The trajectory point. Initialized with current values
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.push_back( shoulder_pan_joint );
    trajectory_point.positions.push_back( shoulder_lift_joint );
    trajectory_point.positions.push_back( elbow_joint );
    // trajectory_point.positions.push_back( elbow_joint );
    // trajectory_point.positions.push_back( shoulder_lift_joint );
    // trajectory_point.positions.push_back( shoulder_pan_joint );



    trajectory_point.positions.push_back( wrist_1_joint);//-M_PI_2 );
    trajectory_point.positions.push_back( wrist_2_joint- 0.0 );
    trajectory_point.positions.push_back( wrist_3_joint- 0.0 );
    trajectory_point.time_from_start = ros::Duration( positionincrement );
    std::cout<<M_PI_2<<std::endl;
    

    tf::Quaternion quatgoal(goaltransform.orientation.x, goaltransform.orientation.y,goaltransform.orientation.z, goaltransform.orientation.w);
    std::cout<<goaltransform.orientation.x<<" "<<goaltransform.orientation.y<<" "<<goaltransform.orientation.z<<" "<<goaltransform.orientation.w<<std::endl;
    
    Eigen::Vector3d posgoal(goaltransform.position.x, goaltransform.position.y, goaltransform.position.z);

    //tf::vector3 poscurr(trajectory_point.positions);
    // std::cout<<quatgoal<<std::endl;
    // std::cout<<"posgoal: "<<posgoal<<std::endl;

    Eigen::Matrix<double,3,3> R;
    R=quat2rotm(quatgoal);
    Eigen::Matrix<double,4, 4> goalFrame=Eigen::Matrix4d::Identity();
    double* T =new double[16];
    // double* temp;
    // temp=T;
     // std::cout<<R<<std::endl;
    goalFrame.block(0,0,3,3)<<R;
    
    goalFrame.block(0,3,3,1)<<posgoal;
    std::cout<<goalFrame<<std::endl;
    std::cout<<goalFrame(0,3)<<std::endl;
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
      {
        // std::cout<<"goalframe value: "<<goalFrame(i,j);
        *T=goalFrame(i,j);
        // std::cout<<" value: "<<*T<<std::endl;
        T++;
      }
    }
    
    Eigen::VectorXf q_curr(6);
    // std::cout<<"here"<<std::endl;
    for(int i=0;i<6;i++)
      q_curr(i)=(trajectory_point.positions[i]);

    // std::cout<<"reached here"<<std::endl;
    
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
      {
        
        T--;
      }
    }

    // T=temp;
    

    // std::cout<<"reached here"<<std::endl;

    double* q_sols=new double[8*6];
    int numOfSol=0;
    numOfSol=ur_kinematics::inverse(T, q_sols, 0);
    std::cout<<numOfSol<<" after inverse"<<std::endl;
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
      std::cout<<"solution i: "<<soli<<std::endl;
      // soli=all_q_possible.col(i);
      if((soli-q_curr).norm()<dist)
        {

          dist=(soli-q_curr).norm();
          std::cout<<"dist "<<dist<<std::endl;
          q_goal=soli;
        }
    }
    // double *q_temp=new double[6];
    // for(int i=0;i<6;i++)
    // {
    //   *q_temp=q_goal(i);
    //   q_temp++;
    // }
    // for(int i=0;i<6;i++)
    // {
    //   q_temp--;
    // }
    // double *T_forward=new double[16];
    // ur_kinematics::forward(q_temp, T_forward);
    // for(int i=0;i<16;i++)
    // {

    // }
    // for(int i=0;i<16;i++)
    // {
    //   std::cout<<*T_forward<<std::endl;
    //   T_forward++;
    // }


    std::cout<<"goal "<<q_goal<<std::endl;
    std::cout<<"curr: "<<q_curr<<std::endl;


    // This is the translation left for the trajectory
    

    Eigen::VectorXf translation(6);
    translation=q_goal-q_curr;
    std::cout<<"Translation: "<<translation<<std::endl;

    int subSteps = 100;
    int count=1;
    // std::cout<<"here"<<std::endl;
    //double positionincrement=0.001;
    while(positionincrement<translation.norm()&&numOfSol!=0)
    {
      //std::cout<<"translation norm:"<<translation.norm()<<std::endl;
      // trajectory_point.positions+=q_curr;
      for(int i=0;i<6;i++)
      {
        trajectory_point.positions[i]=q_curr(i);
      }

      //trajectory_point.positions[0]+=q_curr[0];
      trajectory_point.time_from_start += ros::Duration( positionincrement*10 );
      trajectory.points.push_back( trajectory_point );
      q_curr=q_curr+translation*count/subSteps;
     // std::cout<<q_curr<<std::endl<<std::endl;
      translation=q_goal-q_curr;

      //TODO select correct config. minimize norm of goal - curr for all possibles


    }
    std::cout<<trajectory<<std::endl;	
    pub_trajectory.publish(trajectory);

    // delete q_sols;
    // delete T;

 
  }

public:

  Trajectory( ros::NodeHandle& nh ):
    nh( nh ){
    sub_setpoint=nh.subscribe( "setpoint",1,&Trajectory::SetPoint,this );
    sub_jointstate=nh.subscribe( "joint_states",1,&Trajectory::JointState,this );
    pub_trajectory=nh.advertise<control_msgs::FollowJointTrajectoryAction>( "/follow_joint_trajectory", 1 );
  }

private:

  // Inverse a 3x3 matrix
  // input: A 3x3 matrix
  // output: A 3x3 matrix inverse
  // return the determinant inverse
  double Inverse( double A[3][3], double Ainverse[3][3] ){
    
    double determinant = (  A[0][0]*( A[1][1]*A[2][2]-A[2][1]*A[1][2] ) -
			    A[0][1]*( A[1][0]*A[2][2]-A[1][2]*A[2][0] ) +
			    A[0][2]*( A[1][0]*A[2][1]-A[1][1]*A[2][0] ) );
    
    double invdet = 1.0/determinant;
    
    Ainverse[0][0] =  ( A[1][1]*A[2][2] - A[2][1]*A[1][2] )*invdet;
    Ainverse[0][1] = -( A[0][1]*A[2][2] - A[0][2]*A[2][1] )*invdet;
    Ainverse[0][2] =  ( A[0][1]*A[1][2] - A[0][2]*A[1][1] )*invdet;
    
    Ainverse[1][0] = -( A[1][0]*A[2][2] - A[1][2]*A[2][0] )*invdet;
    Ainverse[1][1] =  ( A[0][0]*A[2][2] - A[0][2]*A[2][0] )*invdet;
    Ainverse[1][2] = -( A[0][0]*A[1][2] - A[1][0]*A[0][2] )*invdet;
    
    Ainverse[2][0] =  ( A[1][0]*A[2][1] - A[2][0]*A[1][1] )*invdet;
    Ainverse[2][1] = -( A[0][0]*A[2][1] - A[2][0]*A[0][1] )*invdet;
    Ainverse[2][2] =  ( A[0][0]*A[1][1] - A[1][0]*A[0][1] )*invdet;
    
    return determinant;
    
  }



  Eigen::Matrix<double,3,3> quat2rotm(tf::Quaternion q)
  {
    float qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z();
    std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
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
