


function InverseKinematic(KecX, KexY, KecZ)
  sim.setJointTargetVelocity(Motor1, (-0.3333 * KecX) - (0.5774 * KecY) - (0.22 * KecZ))
  sim.setJointTargetVelocity(Motor2, (0.6667 * KecX) - (0.22 * KecZ))
  sim.setJointTargetVelocity(Motor3, (-0.3333 * KecX) + (0.5774 * KecY) - (0.22 * KecZ))
end


function sysCall_init()

  simROS2.createPublisher('/omnirobot/cmd_vel')


end



function cmdVelCallback(msg)

end

function sysCall_actuation()
  simROS2.createPublisher('/mazebot/odom', 'nav_msgs/Odometry')
  rpName = simROS2.createPublisher('/odom', 'nav_msgs/Odometry')
  tfName = simROS2.createPublisher('tf', 'geometry_msgs/msg/TransformStamped')
  cmdTopic = simROS2.createSubscription('/mazebot/cmd_vel', 'geometry_msgs/msg/Twist', 'cmdVelCallback')
end


function sysCall_cleanup()
  simROS2.shutdownPublisher('/omnirobot/odom')
  simROS2.shutdownPublisher('/odom')
  simROS2.shutdownPublisher('tf')
  simROS2.shutdownPublisher('/omnirobot/cmd_vel')
  simROS2.shutdownPublisher('orientation')
end