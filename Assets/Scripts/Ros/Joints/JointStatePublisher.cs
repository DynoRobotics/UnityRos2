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

using System.Collections.Generic;

using rclcs;

public class JointStatePublisher : BehaviourNode
{
    public string JointStateTopicName = "/joint_states";
    public override string NodeName { get { return "joint_state_publisher"; } }

    private rclcs.Publisher<sensor_msgs.msg.JointState> jointStatePublisher;
    private sensor_msgs.msg.JointState jointStateMsg;
    private rclcs.Clock clock;

    void Start()
    {
        jointStateMsg = new sensor_msgs.msg.JointState();
        jointStatePublisher = node.CreatePublisher<sensor_msgs.msg.JointState>(JointStateTopicName);
        clock = new rclcs.Clock();
    }

    void Update()
    {
        List<string> jointNames = new List<string>();
        List<double> jointPositions = new List<double>();
        List<double> jointVelocity = new List<double>();
        List<double> jointEffort = new List<double>();

        foreach (JointInterface jointInterface in GetComponentsInChildren<JointInterface>())
        {
            jointNames.Add(jointInterface.JointName);
            jointPositions.Add(jointInterface.Position);
            jointVelocity.Add(jointInterface.Velocity);
            jointEffort.Add(jointInterface.Effort);
        }

        jointStateMsg.name = jointNames;
        jointStateMsg.position = jointPositions;
        jointStateMsg.velocity = jointVelocity;
        jointStateMsg.effort = jointEffort;

        jointStateMsg.header.Update(clock);
        jointStatePublisher.Publish(jointStateMsg); 
    }
}
