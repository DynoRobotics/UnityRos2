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
public class TwistBaseController : MonoBehaviourRosNode
{
    public string NodeName = "twist_base_controller";
    public string CommandVelocityTopic = "cmd_vel";

    public Rigidbody BaseRigidbody;

    public Vector3 commandVelocityLinear = Vector3.zero;
    public Vector3 commandVelocityAngular = Vector3.zero;

    protected override string nodeName { get { return NodeName; } }

    private Subscription<geometry_msgs.msg.Twist> commandVelocitySubscription;

    protected override void StartRos()
    {
        commandVelocitySubscription = node.CreateSubscription<geometry_msgs.msg.Twist>(
            CommandVelocityTopic,
            (msg) =>
            {
                commandVelocityLinear = msg.Linear.Ros2Unity(); 
                commandVelocityAngular = msg.Angular.Ros2Unity(); 
            });
    }

    private void Start()
    {
        if (BaseRigidbody == null)
        {
            BaseRigidbody = GetComponent<Rigidbody>();
        }

        commandVelocityLinear = Vector3.zero;
        commandVelocityAngular = Vector3.zero;

    }

    private void FixedUpdate()
    {
        SpinSome();

        Vector3 deltaPosition = commandVelocityLinear * Time.fixedDeltaTime;
        deltaPosition = BaseRigidbody.transform.TransformDirection(deltaPosition);
        Quaternion deltaRotation = Quaternion.Euler(-commandVelocityAngular * Mathf.Rad2Deg * Time.fixedDeltaTime);

        BaseRigidbody.MovePosition(BaseRigidbody.position + deltaPosition);
        BaseRigidbody.MoveRotation(BaseRigidbody.rotation * deltaRotation);
    }
}
