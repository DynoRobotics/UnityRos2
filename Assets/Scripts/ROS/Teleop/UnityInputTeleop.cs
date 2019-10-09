/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class UnityInputTeleop : MonoBehaviourRosNode
{
    public string NodeName = "unity_teleop";
    public string CommandVelocityTopic = "cmd_vel";

    public float MaxForwardVelocity = 1.0f;
    public float MaxSidewaysVelocity = 1.0f;
    public float MaxRotationalVelocity = 3.0f;

    public float PublishingFrequency = 20.0f;

    public bool UseHolonomicControls = false;
    protected override string nodeName { get { return NodeName; } }

    private Publisher<geometry_msgs.msg.Twist> cmdVelPublisher;
    private geometry_msgs.msg.Twist cmdVelMsg;

    private bool wasPublishing;
    
    protected override void StartRos()
    {
        wasPublishing = false;
        cmdVelPublisher = node.CreatePublisher<geometry_msgs.msg.Twist>(CommandVelocityTopic);
        cmdVelMsg = new geometry_msgs.msg.Twist();
        StartCoroutine("PublishCommandVelocity");
    }

    IEnumerator PublishCommandVelocity()
    {
        for (;;)
        {
            if (ShouldPublishCmdVel())
            {
                cmdVelPublisher.Publish(cmdVelMsg);
                wasPublishing = true;
            } else if (wasPublishing)
            {
                wasPublishing = false;
                cmdVelPublisher.Publish(new geometry_msgs.msg.Twist());
            }
            yield return new WaitForSeconds(1.0f / PublishingFrequency);
        }
    }

    private bool ShouldPublishCmdVel()
    {
        return cmdVelMsg.Angular.Z != 0.0f || cmdVelMsg.Linear.X != 0.0f || cmdVelMsg.Linear.Y != 0.0f;
    }

    private void Update()
    {
        cmdVelMsg.Linear.X = Input.GetAxis("Vertical") * MaxForwardVelocity;
        if (UseHolonomicControls)
        {
            cmdVelMsg.Linear.Y = -Input.GetAxis("Horizontal") * MaxSidewaysVelocity;
            cmdVelMsg.Angular.Z = -Input.GetAxis("Turning") * MaxRotationalVelocity;
        }
        else
        {
            cmdVelMsg.Angular.Z = -Input.GetAxis("Horizontal") * MaxRotationalVelocity;
        }
    }
}
