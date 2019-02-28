/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

you may not use this file except in compliance with the License.
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

public class PoseStampedPublisher : BehaviourNode
{
    public string nodeName = "pose_stamped_publisher";
    public override string NodeName { get { return nodeName; } }

    public string TopicName = "/pose_stamped";

    public int PublisherFrequency = 10;

    public string ParetFrameId = "";
    public Transform ParentFrame;

    public Transform ChildFrame;

    private rclcs.Publisher<geometry_msgs.msg.PoseStamped> publisher;
    private geometry_msgs.msg.PoseStamped message;

    private rclcs.Clock clock = new rclcs.Clock();
    private RosTime lastPublishTime;

    void Start()
    {
        if (ChildFrame == null)
        {
            ChildFrame = transform;
        }

        message = new geometry_msgs.msg.PoseStamped();
        message.header.frame_id = ParetFrameId;
        publisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>(TopicName);
        lastPublishTime = clock.Now;
    }

    new void FixedUpdate()
    {
        if (lastPublishTime.Delay(1.0f / PublisherFrequency).IsInThePast)
        {
            lastPublishTime = clock.Now;
            Vector3 unityPosition = Quaternion.Inverse(ParentFrame.rotation)*(ChildFrame.position - ParentFrame.position);
            message.pose.position.Unity2Ros(unityPosition);
            //TODO(sam): Figure out why z is y in numpy quaternion
            message.pose.orientation.Unity2Ros( Quaternion.Inverse(ParentFrame.rotation) * ChildFrame.rotation);
            message.header.Update(clock);
            publisher.Publish(message);
        }
    }
}
