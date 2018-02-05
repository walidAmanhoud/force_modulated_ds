#include "ForceModulatedTask.h"

ForceModulatedTask* ForceModulatedTask::me = NULL;

ForceModulatedTask::ForceModulatedTask(ros::NodeHandle &n, double frequency):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
// _fmds(originalDynamics)
{
  me = this;
  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.046f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _taskAttractor << -0.5f, 0.0f, 0.25f;
  
  _planeNormal << 0.0f, 0.0f, 1.0f;
  _p << 0.0f,0.0f,0.186f;

  _Fc.setConstant(0.0f);
  _minFc = 0.0f;
  _maxFc = 0.0f;
  _e1 << 0.0f, 0.0f, -1.0f;

  _firstRealPoseReceived = false;
  _firstWrenchReceived = false;
  _wrenchBiasOK = false;
  _stop = false;
  _useOptitrack = false;

  if(_useOptitrack)
  {
    _firstPlane1Pose = false;
    _firstPlane2Pose = false;
    _firstPlane3Pose = false;
    _firstRobotBasisPose = false;    
  }
  else
  {
    _firstPlane1Pose = true;
    _firstPlane2Pose = true;
    _firstPlane3Pose = true;
    _firstRobotBasisPose = true;   
  }

  _lambda1 = 0.0f;
  _lambda2 = 0.0f;

	_firstEventReceived = false;
	_msgFootMouse.event = foot_interfaces::FootMouseMsg::FM_NONE;
	_lastEvent = foot_interfaces::FootMouseMsg::FM_NONE;
	_buttonPressed = false;
  _usedForTraining = false;

  _msgSurfaceMarker.header.frame_id = "world";
  _msgSurfaceMarker.header.stamp = ros::Time();
  _msgSurfaceMarker.ns = "marker_test_triangle_list";
  _msgSurfaceMarker.id = 0;
  _msgSurfaceMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  _msgSurfaceMarker.action = visualization_msgs::Marker::ADD;
  _msgSurfaceMarker.pose.position.x = _p(0);
  _msgSurfaceMarker.pose.position.y = _p(1);
  _msgSurfaceMarker.pose.position.z = _p(2);
  _msgSurfaceMarker.pose.orientation.x = 0.0;
  _msgSurfaceMarker.pose.orientation.y = 1.0;
  _msgSurfaceMarker.pose.orientation.z = 0.0;
  _msgSurfaceMarker.pose.orientation.w = 0.0;
  _msgSurfaceMarker.scale.x = 1.0;
  _msgSurfaceMarker.scale.y = 1.0;
  _msgSurfaceMarker.scale.z = 1.0;
  _msgSurfaceMarker.color.a = 1.0;

  geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  float objectWidth = 0.59f;
  float objectLength = 0.82f;
  p1.x = objectWidth/2.0f;
  p1.y = -objectLength/2.0f;
  p1.z = 0.0f;
  p2.x = -objectWidth/2.0f;
  p2.y = -objectLength/2.0f;
  p2.z = 0.0f;
  p3.x = -objectWidth/2.0f;
  p3.y = objectLength/2.0f;
  p3.z = 0.0f;
  p4.x = -objectWidth/2.0f;
  p4.y = objectLength/2.0f;
  p4.z = 0.0f;
  p5.x = objectWidth/2.0f;
  p5.y = objectLength/2.0f;
  p5.z = 0.0f;
  p6.x = objectWidth/2.0f;
  p6.y = -objectLength/2.0f;
  p6.z = 0.0f;

  Eigen::Vector3f t1,t2;
  t1 << 1.0f,0.0f,0.0f;
  t2 << 0.0f,1.0f,0.0f;
  _p1 = _p-0.3f*t1+(objectLength/2.0f)*t2;
  _p2 = _p1-objectLength*t2;
  _p3 = _p1-objectWidth*t1;

  std_msgs::ColorRGBA c;
  c.r = 0.7;
  c.g = 0.7;
  c.b = 0.7;
  c.a = 1.0;

  for(int k = 0; k < 6; k++)
  {
    _msgSurfaceMarker.colors.push_back(c);
  }

  _msgSurfaceMarker.points.push_back(p1);
  _msgSurfaceMarker.points.push_back(p2);
  _msgSurfaceMarker.points.push_back(p3);
  _msgSurfaceMarker.points.push_back(p4);
  _msgSurfaceMarker.points.push_back(p5);
  _msgSurfaceMarker.points.push_back(p6);


  _msgArrowMarker.header.frame_id = "world";
  _msgArrowMarker.header.stamp = ros::Time();
  _msgArrowMarker.ns = "marker_test_arrow";
  _msgArrowMarker.id = 1;
  _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  p1.x = _p(0);
  p1.y = _p(1);
  p1.z = _p(2);
  p2.x = _p(0)+0.3f*_e1(0);
  p2.x = _p(1)+0.3f*_e1(1);
  p2.x = _p(2)+0.3f*_e1(2);
  _msgArrowMarker.scale.x = 0.1;
  _msgArrowMarker.scale.y = 0.3;
  _msgArrowMarker.scale.z = 0.1;
  _msgArrowMarker.color.a = 1.0;
  _msgArrowMarker.color.r = 1.0;
  _msgArrowMarker.color.g = 0.0;
  _msgArrowMarker.color.b = 0.0;
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);

  _fmds.setOriginalDynamics(boost::bind(&ForceModulatedTask::originalDynamics, this, _1));

  _outputFile.open ("src/force_modulated_ds/learning_data.txt");
  _previousTime = ros::Time::now().toSec();

}


bool ForceModulatedTask::init() 
{
  // Subscriber definitions
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &ForceModulatedTask::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  // _subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &ForceModulatedTask::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/joint_controllers/twist", 1, &ForceModulatedTask::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ForceModulatedTask::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackRobotBasisPose = _n.subscribe("/optitrack/robot/pose", 1, &ForceModulatedTask::updateRobotBasisPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane1Pose = _n.subscribe("/optitrack/plane1/pose", 1, &ForceModulatedTask::updatePlane1Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane2Pose = _n.subscribe("/optitrack/plane2/pose", 1, &ForceModulatedTask::updatePlane2Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane3Pose = _n.subscribe("/optitrack/plane3/pose", 1, &ForceModulatedTask::updatePlane3Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subFootMouse= _n.subscribe("/foot_mouse", 1, &ForceModulatedTask::updateFootMouseData, this, ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1); 
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ForceModulatedTask/taskAttractor", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ForceModulatedTask/markers", 10);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("ForceModulatedTask/forceNorm", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ForceModulatedTask/filteredWrench", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ForceModulatedTask::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ForceModulatedTask::stopNode);

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The force modulated task is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void ForceModulatedTask::run()
{
  while (!_stop) 
  {
    if(_firstRealPoseReceived && _wrenchBiasOK)
    // if(_firstRealPoseReceived && _wrenchBiasOK &&//)// &&
    //    _firstRobotBasisPose && _firstPlane1Pose &&
    //    _firstPlane2Pose && _firstPlane3Pose)
    if(_firstRealPoseReceived)
    {

      _mutex.lock();
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _Fc.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  
  ros::shutdown();
}


void ForceModulatedTask::stopNode(int sig)
{
  me->_stop = true;
}


void  ForceModulatedTask::computeCommand()
{
  modulatedDynamics();
}


Eigen::Vector3f ForceModulatedTask::originalDynamics(Eigen::Vector3f position)
{
  Eigen::Vector3f velocity;

  position = position-_taskAttractor;

  velocity(2) = -position(2);

  float R = sqrt(position(0) * position(0) + position(1) * position(1));
  float T = atan2(position(1), position(0));

  float r = 0.05f;
  float omega = M_PI;

  velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
  velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

  return velocity;
}


void ForceModulatedTask::modulatedDynamics()
{

  double currentTime = ros::Time::now().toSec();

  // Extract linear speed, force and torque data
  Eigen::Vector3f v = _twist.segment(0,3);
  Eigen::Vector3f force = _filteredWrench.segment(0,3);  
  Eigen::Vector3f torque = _filteredWrench.segment(3,3);

  _vd = originalDynamics(_x);

  // Bound desired velocity  
  if(_vd.norm()>0.3f)
  {
    _vd *= 0.3f/_vd.norm();
  }

  Eigen::Vector3f vdm;
  float forceValue = std::max(0.0f,_wRb.col(2).dot(-force));
  Eigen::Matrix3f M;
  M.setIdentity();

  _usedForTraining = false;
  if(_buttonPressed)
  {
    if(currentTime-_previousTime>0.005f && forceValue>3.0f)
    {
      _count++;
      _fmds.addData(_x,_wRb.col(2),forceValue/_lambda1);
      _previousTime = currentTime;
      _usedForTraining = true;
    }
  }
  else
  {
    if(_count>500)
    {
      vdm = _fmds.getModulatedDynamics(_x);
      _e1 = _fmds.getEstimatedNormal();
      M = _fmds.getModulationMatrix();
    }
  }

  std::cerr << "Count: " << _count << "force: " << forceValue << std::endl;
  std::cerr << "e1: " << _e1.transpose() << std::endl;

  _controller.updateDampingGains(_lambda1,_lambda2);

  Eigen::Vector3f Ftemp = _controller.step(_vd,v);
  _Fc = Ftemp;

  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Matrix3f K;
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  K << getSkewSymmetricMatrix(k);

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Convert rotation error into axis angle representation
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);

  // Compute final quaternion on plane
  Eigen::Vector4f qf = quaternionProduct(qtemp,_q);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  _qd = slerpQuaternion(_q,qf,1.0f);

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp;
}


Eigen::Vector4f ForceModulatedTask::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f ForceModulatedTask::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f ForceModulatedTask::rotationMatrixToQuaternion(Eigen::Matrix3f R)
{
  Eigen::Vector4f q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);

  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


Eigen::Matrix3f ForceModulatedTask::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void ForceModulatedTask::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));
}


Eigen::Vector4f ForceModulatedTask::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
{

  Eigen::Vector4f q;

  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
  if(q1.dot(q2)<0.0f)
  {   
    q2 = -q2;
  }

  float dotProduct = q1.dot(q2);
  if(dotProduct > 1.0f)
  {
    dotProduct = 1.0f;
  }
  else if(dotProduct < -1.0f)
  {
    dotProduct = -1.0f;
  }

  float omega = acos(dotProduct);

  if(std::fabs(omega)<FLT_EPSILON)
  {
    q = q1.transpose()+t*(q2-q1).transpose();
  }
  else
  {
    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
  }

  return q;
}


void ForceModulatedTask::processFootMouseData(void)
{
  uint8_t event;
  int buttonState;
  bool newEvent = false;
  // If new event received update last event, otherwhise keep the last one

  if(_msgFootMouse.event > 0)
  {
    _lastEvent = _msgFootMouse.event;
    buttonState = _msgFootMouse.buttonState;
    newEvent = true;
  }
  else
  {
    buttonState = 0;
    newEvent = false;
  }

  event = _lastEvent;

  // Process corresponding event
  switch(event)
  {
    case foot_interfaces::FootMouseMsg::FM_BTN_A:
    {
      processABButtonEvent(buttonState,newEvent,-1.0f);
      break;
    }
    case foot_interfaces::FootMouseMsg::FM_BTN_B:
    {
      processABButtonEvent(buttonState,newEvent,1.0f);
      break;
    }
    case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
    {
      processRightClickEvent(buttonState,newEvent);
      break;
    }
    default:
    {
      break;
    }
  }
}


void ForceModulatedTask::processABButtonEvent(int value, bool newEvent, int direction)
{
  if(newEvent)
  {
    if(value>0) // Button pressed
    {
      _buttonPressed = true;
    }
    else
    {
      _buttonPressed = false;
    }
  }
}


void ForceModulatedTask::processRightClickEvent(int value, bool newEvent)
{
  if(newEvent)
  {
    if(value>0) // Button pressed
    {
      _buttonPressed = true;
    }
    else
    {
      _buttonPressed = false;
    }
  }
}



void ForceModulatedTask::publishData()
{
  // _mutex.lock();

  // Publish desired pose
  _msgDesiredPose.position.x = _xd(0);
  _msgDesiredPose.position.y = _xd(1);
  _msgDesiredPose.position.z = _xd(2);
  _msgDesiredPose.orientation.w = _qd(0);
  _msgDesiredPose.orientation.x = _qd(1);
  _msgDesiredPose.orientation.y = _qd(2);
  _msgDesiredPose.orientation.z = _qd(3);

  _pubDesiredPose.publish(_msgDesiredPose);

  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubTaskAttractor.publish(_msgTaskAttractor);

  _msgSurfaceMarker.header.frame_id = "world";
  _msgSurfaceMarker.header.stamp = ros::Time();
  Eigen::Vector3f center;
  if(_useOptitrack)
  {
    center = _p1+0.5f*(_p2-_p1)+0.5f*(_p3-_p1); 
  }
  else
  {
    center << -0.4f, 0.0f, 0.186f;
  }
  _msgSurfaceMarker.pose.position.x = center(0);
  _msgSurfaceMarker.pose.position.y = center(1);
  _msgSurfaceMarker.pose.position.z = center(2);
  Eigen::Vector3f u,v,n;
  u = _p3-_p1;
  v = _p2-_p1;
  u /= u.norm();
  v /= v.norm();
  n = u.cross(v);
  Eigen::Matrix3f R;
  R.col(0) = u;
  R.col(1) = v;
  R.col(2) = n;
  Eigen::Vector4f q = rotationMatrixToQuaternion(R);


  _msgSurfaceMarker.pose.orientation.x = q(1);
  _msgSurfaceMarker.pose.orientation.y = q(2);
  _msgSurfaceMarker.pose.orientation.z = q(3);
  _msgSurfaceMarker.pose.orientation.w = q(0);

  _pubMarker.publish(_msgSurfaceMarker);

  _msgArrowMarker.points.clear();
  geometry_msgs::Point p1, p2;
  p1.x = _x(0);
  p1.y = _x(1);
  p1.z = _x(2);
  p2.x = _x(0)+0.3f*_e1(0);
  p2.y = _x(1)+0.3f*_e1(1);
  p2.z = _x(2)+0.3f*_e1(2);
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
  _pubMarker.publish(_msgArrowMarker);

  std_msgs::Float32 msg;
  msg.data = _forceNorm;
  _pubForceNorm.publish(msg);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);


  _msgDesiredWrench.force.x = _Fc(0);
  _msgDesiredWrench.force.y = _Fc(1);
  _msgDesiredWrench.force.z = _Fc(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);
}

void ForceModulatedTask::logData()
{
  _outputFile << ros::Time::now() << " " << (int) _usedForTraining 
              << " " << _x(0) << " " << _x(1) << " " << _x(2) 
              << " " << _wRb(0,2) << " " << _wRb(1,2) << " " << _wRb(2,2) 
              << " " << _filteredWrench(0) << " " << _filteredWrench(1) << " " << _filteredWrench(2)
              << " " << std::endl;
}

void ForceModulatedTask::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if(!_firstRealPoseReceived)
  {
    _firstRealPoseReceived = true;
    _xd = _x;
    _qd = _q;
    _x0 = _x;
    _vd.setConstant(0.0f);
  }
}

 
void ForceModulatedTask::updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRealPoseReceived)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRealPoseReceived)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}


void ForceModulatedTask::updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;
}


void ForceModulatedTask::updateRobotBasisPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _robotBasisPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  _robotBasisPosition(2) -= 0.025f;
  if(!_firstRobotBasisPose)
  {
    _firstRobotBasisPose = true;
  }
}


void ForceModulatedTask::updatePlane1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane1Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane1Pose)
  {
    _firstPlane1Pose = true;
  }
}


void ForceModulatedTask::updatePlane2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane2Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane2Pose)
  {
    _firstPlane2Pose = true;
  }
}


void ForceModulatedTask::updatePlane3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane3Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane3Pose)
  {
    _firstPlane3Pose = true;
  }
}


void ForceModulatedTask::updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg)
{
	_msgFootMouse = *msg;

	if(!_firstEventReceived && _msgFootMouse.event > 0)
	{
		_firstEventReceived = true;
	}
}


void ForceModulatedTask::dynamicReconfigureCallback(force_modulated_ds::forceModulatedTask_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _contactForceThreshold = config.contactForceThreshold;
  _targetForce = config.targetForce;
  _polishing = config.polishing;
  _linear = config.linear;
  _controlForce = config.controlForce;
  _k1 = config.k1;
  _k2 = config.k2;
  _minFc = config.minFc;
  _maxFc = config.maxFc;
  _vInit = config.vInit;
  _lambda1 = config.lambda1;
  _lambda2 = config.lambda2;
  _controller.updateDampingGains(_lambda1,_lambda2);
}
