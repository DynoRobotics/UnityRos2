/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

[RequireComponent(typeof(Rigidbody))]
public class TurtlebotWheelSpinner : MonoBehaviourRosNode
{
    public string NodeName = "turtlebot_wheel_spinner";
    public string CommandVelocityTopic = "cmd_vel";

    public Transform LeftWheel;
    public Transform RightWheel;

    public float WheelRadius = 0.0f;
    private float WheelBase = 0.0f;

    private float leftWheelAngularVelocity;
    private float rightWheelAngularVelocity;

    protected override string nodeName { get { return NodeName; } }

    private Subscription<geometry_msgs.msg.Twist> commandVelocitySubscription;

    protected override void StartRos()
    {
         if (LeftWheel != null && RightWheel != null)
        {
            // TODO(sam): figure out a better way of finding wheel radius...
            WheelRadius = LeftWheel.position.y - transform.position.y;
            WheelBase = Vector3.Distance(LeftWheel.position, RightWheel.position);
        }

        commandVelocitySubscription = node.CreateSubscription<geometry_msgs.msg.Twist>(
            CommandVelocityTopic,
            (msg) =>
            {
                Vector3 localVelocity = msg.Linear.Ros2Unity(); 
                Vector3 localAngularVelocity = msg.Angular.Ros2Unity(); 
                leftWheelAngularVelocity = -(localVelocity.z - localAngularVelocity.y * WheelBase / 2.0f) / WheelRadius;
                rightWheelAngularVelocity = -(localVelocity.z + localAngularVelocity.y * WheelBase / 2.0f) / WheelRadius;
                // Debug.Log(rightWheelAngularVelocity);
            });
    }

    void Update() {
        float radiansToDegrees = 180/Mathf.PI;
        RightWheel.localRotation *= Quaternion.Euler(0, radiansToDegrees * rightWheelAngularVelocity * Time.deltaTime, 0);
        LeftWheel.localRotation *= Quaternion.Euler(0, radiansToDegrees * leftWheelAngularVelocity * Time.deltaTime, 0);
        SpinSome();
    }
}

