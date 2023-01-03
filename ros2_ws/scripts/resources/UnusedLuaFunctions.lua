function getOdometryTransformStamped(objHandle,name,relTo,relToName)
  linearVel, angularVel = sim.getObjectVelocity(objHandle)
  local arr = {} for i=1,36 do arr[i] = 0 end
  
  return {
      header={
          stamp=simROS2.getTime(),
          frame_id=relToName
      },
      child_frame_id=name,
      pose={
          pose={
              position=sim.getObjectPosition(objHandle, relTo),
              orientation=sim.getObjectQuaternion(objHandle, relTo),
          },
          covariance=arr
      },
      twist={
          twist={
              linear=linearVel,
              angular=angularVel
          },
          covariance=arr
      }
  }
end



function getObjectPoseStamped(objHandle, name, relTo)
  return {
      header={
          stamp=simROS2.getTime(),
          frame_id=name,
      },
      pose={
          position=sim.getObjectPosition(objHandle, relTo),
          orientation=sim.getObjectQuaternion(objHandle, relTo)
      }
  }
end





function sysCall_actuation()
  -- put your actuation code here

  --do return end

  R = 0.05
  d = 0.16

  p=sim.getObjectPosition(robot,-1)
  o=sim.getObjectOrientation(robot,-1)
  print("p",p)
  --print("o",o)
  k1 = 1;
  u1_io = k1*(-0.5 - p[1]);
  u2_io = k1*(1 - p[2]);
  --print("u2",u2_io)
  --print("o3",o[3])
  o[3] = o[3] + math.pi/2
  v = math.cos(o[3])*u1_io + math.sin(o[3])*u2_io;
  w = -math.sin(o[3])*u1_io/0.01 + math.cos(o[3])*u2_io/0.01;
  
  v = -v
  w = -w
  --v = 0
  --w = 1
  wl = (2*v - w*d)/(2*R)
  wr = (2*v + w*d)/(2*R)
  sim.setJointTargetVelocity(left_motor,wl)
  sim.setJointTargetVelocity(right_motor,wr)
  
end