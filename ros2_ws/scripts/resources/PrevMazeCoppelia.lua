function sysCall_init()

  robot=sim.getObjectHandle(sim.handle_self) -- Handle of the robot
  map = sim.getObjectHandle('/Maze')
  finish_line=sim.getObjectHandle('/Maze/finish_line')
  left_motor=sim.getObjectHandle("/PioneerP3DX/leftMotor") -- Handle of the robot
  right_motor=sim.getObjectHandle("/PioneerP3DX/rightMotor") -- Handle of the robot    
  cfj1=sim.getObjectHandle("/PioneerP3DX/caster_freeJoint1")
  cfj2=sim.getObjectHandle("/PioneerP3DX/caster_freeJoint2")
  v = 0.1
  w = 0.1

  motorSub=simROS2.createSubscription('/cmd_vel','geometry_msgs/msg/Twist','setMotorVelocity_cb')
  
  
  jointStatesPub = simROS2.createPublisher('/joint_states', 'sensor_msgs/msg/JointState')
  
  --robot_desc_pub = simROS2.createPublisher('/robot_description', 'std_msgs/msg/String')
  --simROS2.publish(robot_desc_pub, {data='mazebot'})
  
  
  --simROS2.sendTransform(getTransformStamped(robot,'odom',map,'map'))
  --simROS2.sendTransform(getTransformStamped(robot,'base_link',robot,'odom'))
  --simROS2.sendTransform(getTransformStamped(robot,'base_link',map,'map'))
  
end

function setMotorVelocity_cb(msg)
  print('RECEIVED TWIST MESSAGE!!!', msg)
  --print(msg.linear.x)
  --print(msg.linear.y)
  --print(msg.angular.z)
  R = 0.05
  d = 0.16

  --sim.setObjectOrientation(robot,-1,orientation)
  --sim.setObjectPosition(robot,-1,position)
  v = -msg.linear.x/4
  w = -msg.angular.z/4
  wl = (2*v - w*d)/(2*R)
  wr = (2*v + w*d)/(2*R)
  sim.setJointTargetVelocity(left_motor,wl)
  sim.setJointTargetVelocity(right_motor,wr)
  
end

function sysCall_actuation()
  -- put your actuation code here
  simROS2.publish(jointStatesPub, getJointStates('base_link', left_motor, right_motor, cfj1, cfj2))
end

function getTransformStamped(objHandle,name,relTo,relToName)
  
  p=sim.getObjectPosition(objHandle,relTo)
  o=sim.getObjectQuaternion(objHandle,relTo)
  
  return {
      header={
          stamp=simROS2.getTime(),
          frame_id=relToName
      },
      child_frame_id=name,
      transform={
          translation={x=p[1],y=p[2],z=p[3]},
          rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
      }
  }
end

function getPoint(objHandle, relTo)
  p=sim.getObjectPosition(objHandle,relTo)
  return {
      x=p[0],
      y=p[1],
      z=p[2]
  }
end


function getJointStates(relToName, leftHandle, rightHandle, cfj1, cfj2)
  return {
      header={
          stamp=simROS2.getTime(),
          frame_id=relToName
      },
      name={ sim.getObjectAlias(leftHandle), sim.getObjectAlias(rightHandle), sim.getObjectAlias(cfj1), sim.getObjectAlias(cfj2) },
      position={ sim.getJointPosition(leftHandle), sim.getJointPosition(rightHandle), sim.getJointPosition(cfj1), sim.getJointPosition(cfj2) },
      velocity={ sim.getJointVelocity(leftHandle), sim.getJointVelocity(rightHandle), sim.getJointVelocity(cfj1), sim.getJointVelocity(cfj2) },
      effort={ sim.getJointForce(leftHandle), sim.getJointForce(rightHandle), sim.getJointForce(cfj1), sim.getJointForce(cfj2) }
  }
end
