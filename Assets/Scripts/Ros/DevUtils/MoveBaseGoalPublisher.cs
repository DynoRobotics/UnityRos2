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

public class MoveBaseGoalPublisher : BehaviourNode
{
    public string nodeName = "move_base_goal_publisher";
    public override string NodeName { get { return nodeName; } }

    public string TopicName = "/move_base_simple/goal";

    private Clock clock = new Clock();
    private Publisher<geometry_msgs.msg.PoseStamped> publisher;
    private geometry_msgs.msg.PoseStamped moveBaseSimpleMessage;

    void Start()
    {
        publisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>(TopicName);
        moveBaseSimpleMessage = new geometry_msgs.msg.PoseStamped();
    }

    public void PublishNavigationGoal()
    {
        moveBaseSimpleMessage.header.Update(clock);
        moveBaseSimpleMessage.pose.LocalUnity2Ros(transform);
        publisher.Publish(moveBaseSimpleMessage);
    }
}
