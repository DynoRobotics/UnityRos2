/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using rclcs;

public class UnityInputTeleop : BehaviourNode
{
    public override string NodeName { get { return "unity_input_teleop"; } }

    public string TwistTopic = "/cmd_vel";
    public float MaxForwardVelocity = 0.5f;
    public float MaxSidewaysVelocity = 0.3f;
    public float MaxAngularVelocity = 1.0f;

    public bool UseHolonomicControls = false;

    private Publisher<geometry_msgs.msg.Twist> publisher;
    private geometry_msgs.msg.Twist twistMsg;

    private bool isPublishing;

    void Start()
    {
        twistMsg = new geometry_msgs.msg.Twist();
        publisher = node.CreatePublisher<geometry_msgs.msg.Twist>(TwistTopic);
    }

    void Update()
    {
        var linear = twistMsg.linear;
        var angular = twistMsg.angular;

        linear.x = Input.GetAxis("Vertical") * MaxForwardVelocity;

        if (UseHolonomicControls)
        {
            linear.y = -Input.GetAxis("Horizontal") * MaxSidewaysVelocity;
            angular.z = -Input.GetAxis("Turning") * MaxAngularVelocity;
        }
        else
        {
            angular.z = -Input.GetAxis("Horizontal") * MaxAngularVelocity;
        }

        if (angular.z != 0.0f || linear.x != 0.0f || linear.y != 0.0f)
        {
            publisher.Publish(twistMsg);
            isPublishing = true;
        }
        else if (isPublishing)
        {
            linear.x = 0.0f;
            angular.z = 0.0f;
            publisher.Publish(twistMsg);
            isPublishing = false;
        }
    }
}
