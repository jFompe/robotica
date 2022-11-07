-- base_sensor child script
-----------------
if (sim_call_type == sim_childscriptcall_initialization) then
  laserHandle = simGetObjectHandle('somename')
  jointHandle = simGetObjectHandle('somename')
  modelRef = simGetObjectHandle('somename')
  modelHandle = siemGetObjectAssociatedWithScript(sim_handle_self)
  objName = simGetObjectName(modelHandle)
  communicationTube = simTubeOpen(0, objName..'_suffix', 1)

  scanRange = 180 * math.pi / 180
  stepSize = 2 * math.pi / 1024
  pts = math.floor(scanRange / stepSize)
  dists = {}
  points = {}
  segments = {}

  for i=1, pts*3, 1 do
    table.insert(points, 0)
  end
  for i=1, pts*7, 1 do
    table.insert(segments, 0)
  end

  black = {0, 0, 0}
  red = {1, 0, 0}
  lines100 = simAddDrawingObject(sim_drawing_lines, 1, 0, -1, 1000, black, black, black, red)
  points100 = simAddDrawingObject(sim_drawing_points, 4, 0, -1, 1000, black, black, black, red)

  -- Check ROS loaded and stuff


  topicName = simExtROS_enablePublisher('front_scan', 1, simros_strmcmd_get_laser_scanner_data, modelHandle, ??)

  parentTf = simGetObjectHandle('parentTf#')
  tfname = simExtROS_enablePublisher('tf', 1, simros_strmcmd_get_transform, modelHandle, parentTf, ??)
end


if (sim_call_type == sim_childscriptcall_cleanup) then
  simRemoveDrawingObject(lines100)
  simRemoveDrawingObject(points100)
end


if (sim_call_type == sim_childscriptcall_sensing) then
  showLaserPoints = simGetScriptSimulationParameter(sim_handle_self, 'showLaserPoints')
  showLaserSegments = simGetScriptSimulationParameter(sim_handle_self, 'showLaserSegments')
  dists = {}
  angle = -scanRange*0.5
  simSetJointPosition(jointHandle, angle)
  jointPos = angle

  laserOrigin = simGetObjectPosition(jointHandle, -1)
  modelInverseMatrix = simGetInvertedMatrix(simGetObjectMatrix(modelRef, -1))

  for ind=0, pts-1, 1 do

    r, dist, pt = simHandleProximitySensor(laserHandle)
    m = simGetObjectHandle(laserHandle, -1)
    if r>0 then
      dists[ind] = dist
      ptAbsolute = simMultiplyVector(m, pt)
      ptRelative = simMultiplyVector(modelInverseMatrix, ptAbsolute)
      points[3*ind+1] = ptRelative[1]
      points[3*ind+2] = ptRelative[2]
      points[3*ind+3] = ptRelative[3]
      segments[7*ind+7] = 1 -- valid point
    else:
      dists[ind] = 0
      ptAbsolute = simMultiplyVector(m, {0,0,6})
      points[3*ind+1] = 0
      points[3*ind+2] = 0
      points[3*ind+3] = 0
      segments[7*ind+7] = 0 -- invalid point
    end

    segments[7*ind+1] = laserOrigin[1]
    segments[7*ind+2] = laserOrigin[2]
    segments[7*ind+3] = laserOrigin[3]
    segments[7*ind+4] = ptAbsolute[1]
    segments[7*ind+5] = ptAbsolute[2]
    segments[7*ind+6] = ptAbsolute[3]

    ind = ind + 1
    angle = angle + stepSize
    jointPos = jointPos + stepSize
    simSetJointPosition(jointHandle, jointPos)
  end


  simAddDrawingObjectItem(line100, nil)
  simAddDrawingObjectItem(points100, nil)

  if (showLaserPoints or showLaserSegments) then
    t = {0,0,0,0,0,0}
    for i=0, pts-1, 1, do
      t[1] = segments[7*i+4]
      t[2] = segments[7*i+5]
      t[3] = segments[7*i+6]
      t[4] = segments[7*i+1]
      t[5] = segments[7*i+2]
      t[6] = segments[7*i+3]
      if showLaserSegments then
        simAddDrawingObjectItem(lines100, t)
      end
      if (showLaserPoints and segments[7*i+7] ~= 0) then
        simAddDrawingObjectItem(points100, t)
      end
    end
  end

  -- Send the data
  if #points > 0 then
    table.insert(dists, -scanRange * 0.5) -- append angle min
    table.insert(dists, scanRange * 0.5 - stepSize) -- append angle max
  end
end







--base_link child script
--------------------------
InverseKinematic = function(KecX, KexY, KecZ)
  simSetJointTargetVelocity(Motor1, (-0.3333 * KecX) - (0.5774 * KecY) - (0.22 * KecZ))
  simSetJointTargetVelocity(Motor2, (0.6667 * KecX) - (0.22 * KecZ))
  simSetJointTargetVelocity(Motor3, (-0.3333 * KecX) + (0.5774 * KecY) - (0.22 * KecZ))
end

if (sim_call_type == sim_childscriptcall_initialization) then

  simExtROS_enablePublisher('/omnirobot/cmd_vel', 1, simros_strmcmd_set_twist_command, -1, -1, 'twistSignal')

  Motor1 = simGetObjectHandle('wheel0_joint')
  Motor2 = simGetObjectHandle('wheel1_joint')
  Motor3 = simGetObjectHandle('wheel2_joint')

  robot = simGetObjectHandle('base_link')
end




if (sim_call_type == sim_childscriptcall_actuation) then

  simExtROS_enablePublisher('/omnirobot/odom', 1, simros_strmcmd_get_odom_data, robot, -1, '')
  rpName = simExtROS_enablePublisher('/odom', 1, simros_strmcmd_get_odom_data, robot, -1, '')
  tfname = simExtROS_enablePublisher('tf', 1, simros_strmcmd_get_transform, robot, -1, 'base_link%odom')
  cmdTopic = simExtROS_enableSubscriber('/omnirobot/cmd_vel', 1, simros_strmcmd_set_twist_command, -1, -1, 'twistSignal')

  packedData = simGetStringSignal('twistSignal')
  if (packedData) then
    twistData = simUnpackFloats(packedData)
    InversKinematic(twistData[1] / 100, twistData[2] / 100, twistData[3] / 100)
  end

  orient = simGetObjectOrientation(robot, -1)
  yaw = 90 - orient[3] * 57.324840764
  if (yaw > 180) then
    yaw = yaw - 360
  end
  if (yaw < -180) then
    yaw = yaw + 360
  end


  simAddStatusbarMessage(string.format('',))
  simSetFloatSignal()

  simExtROS_enablePublisher('/orientation', 1, simros_strmcmd_get_float_signal, -1, -1, 'heading')

  position = simGetObjectPosition(robot, -1)
  rotation = simGetObjectQuaternion(robot, -1)
  velocity_linear, velocity_angular = simGetObjectVelocity(robot)

  position_left = simGetJointPosition(Motor1)
  position_right = simGetJointPosition(Motor3)
  position_rear = simGetJointPosition(Motor2)
end




if (sim_call_type == sim_childscriptcall_cleanup) then
  simExtROS_disablePublisher('/omnirobot/odom')
  simExtROS_disablePublisher('/odom')
  simExtROS_disablePublisher('tf')
  simExtROS_disablePublisher('/omnirobot/cmd_vel')
  simExtROS_disablePublisher('orientation')
end