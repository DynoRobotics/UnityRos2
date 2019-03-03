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

public class PoseWithCovarianceStampedPublisher : BehaviourNode
{
    public string nodeName = "pose_with_covariance_stamped_publisher";
    public override string NodeName { get { return nodeName; } }
    public string FrameId = "map";

    public string TopicName = "/pose_with_covariance_stamped";

    public bool UseLocalTransform = true;

    private List<double> poseCovarianceDiagonal = new List<double> { 0.001d, 0.001d, 0.001d, 0.001d, 0.001d, 0.03d };

    private rclcs.Publisher<geometry_msgs.msg.PoseWithCovarianceStamped> publisher;
    private geometry_msgs.msg.PoseWithCovarianceStamped poseWithCovarianceStampedMessage;

    private rclcs.Clock clock = new rclcs.Clock();

    void Start()
    {
        poseWithCovarianceStampedMessage = new geometry_msgs.msg.PoseWithCovarianceStamped();
        poseWithCovarianceStampedMessage.pose.covariance = poseCovarianceDiagonal.CovarianceMatrixFromDiagonal();
        poseWithCovarianceStampedMessage.header.frame_id = FrameId;
        publisher = node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(TopicName);
    }

    void Update()
    {
        poseWithCovarianceStampedMessage.header.Update(clock);
        if (UseLocalTransform)
        {
            poseWithCovarianceStampedMessage.pose.pose.LocalUnity2Ros(transform);
        }
        else
        {
            poseWithCovarianceStampedMessage.pose.pose.Unity2Ros(transform);
        }
         
        publisher.Publish(poseWithCovarianceStampedMessage);
    }

}
