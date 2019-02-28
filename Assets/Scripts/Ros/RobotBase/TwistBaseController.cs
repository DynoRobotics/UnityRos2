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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class TwistBaseController : BehaviourNode
{
    public override string NodeName { get { return "twist_base_controller"; } }

    public string TwistTopic = "/cmd_vel";

    public float LinearForwardVelocity = 0.0f;
    public float LinearRightVelocity = 0.0f;
    public float AngularVelocity = 0.0f;

    public Rigidbody BaseRigidbody;

    void Start()
    {
        Subscription<geometry_msgs.msg.Twist> subscription = node.CreateSubscription<geometry_msgs.msg.Twist>(
            TwistTopic, (msg) => 
            {
                LinearForwardVelocity = (float)msg.linear.x;
                LinearRightVelocity = -(float)msg.linear.y;
                AngularVelocity = -(float)msg.angular.z;
            });
        
        if (BaseRigidbody == null)
        {
            BaseRigidbody = GetComponentInChildren<Rigidbody>();
        }
    }

    new void FixedUpdate()
    {
        Vector3 deltaPosition = (BaseRigidbody.transform.forward * LinearForwardVelocity + 
                                 BaseRigidbody.transform.right * LinearRightVelocity) * Time.fixedDeltaTime;
    
        BaseRigidbody.MovePosition(this.BaseRigidbody.position + deltaPosition);

        Quaternion deltaRotation = Quaternion.Euler(new Vector3(0.0f, AngularVelocity * Mathf.Rad2Deg, 0.0f) * 
                                                    Time.fixedDeltaTime);

        this.BaseRigidbody.MoveRotation(this.BaseRigidbody.rotation * deltaRotation);

        Rclcs.SpinOnce(node, 0.0d);
    }
}
