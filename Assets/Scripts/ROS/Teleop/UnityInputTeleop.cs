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

    public float PublishingFrequency = 20.0f;

    public bool UseHolonomicControls = false;

    private Publisher<geometry_msgs.msg.Twist> cmdVelPublisher;
    private geometry_msgs.msg.Twist cmdVelMsg;
    protected override string nodeName { get { return NodeName; } }

    protected override void StartRos()
    {
        cmdVelPublisher = node.CreatePublisher<geometry_msgs.msg.Twist>(CommandVelocityTopic);
        cmdVelMsg = new geometry_msgs.msg.Twist();
        StartCoroutine("PublishCommandVelocity");
    }

    IEnumerator PublishCommandVelocity()
    {
        for (;;)
        {
            cmdVelMsg.Linear.X = Input.GetAxis("Vertical");
            if (UseHolonomicControls)
            {
                cmdVelMsg.Linear.Y = -Input.GetAxis("Horizontal");
                cmdVelMsg.Angular.Z = -Input.GetAxis("Turning");
            }
            else
            {
                cmdVelMsg.Angular.Z = -Input.GetAxis("Horizontal");
            }
            cmdVelPublisher.Publish(cmdVelMsg);
            yield return new WaitForSeconds(1.0f / PublishingFrequency);
        }
    }
}
