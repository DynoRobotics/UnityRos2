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

public class OdometryPublisher : BehaviourNode
{
    public string OdometryTopic = "/odom";
    public Rigidbody BaseRigidbody;

    public bool PublishTf = true;
    public string TfTopic = "/tf";
    public uint tfPostDateMilliseconds = 0; // post date to prevent "lookup into future" error for sensor data

    public override string NodeName { get { return "odometry_publisher"; } }

    private rclcs.Clock clock;

    private rclcs.Publisher<nav_msgs.msg.Odometry> odometryPublisher;
    private rclcs.Publisher<tf2_msgs.msg.TFMessage> tfPublisher;

    private nav_msgs.msg.Odometry odometryMsg;
    private geometry_msgs.msg.PoseWithCovariance robotPose;
    private geometry_msgs.msg.TwistWithCovariance robotTwist;

    private tf2_msgs.msg.TFMessage tfMsg;
    private geometry_msgs.msg.TransformStamped tfTransformStamped;

    private List<double> poseCovarianceDiagonal = new List<double> { 0.001d, 0.001d, 0.001d, 0.001d, 0.001d, 0.03d };
    private List<double> twistCovarianceDiagonal = new List<double> { 0.001d, 0.001d, 0.001d, 0.001d, 0.001d, 0.03d };

    void Start()
    {
        odometryPublisher = node.CreatePublisher<nav_msgs.msg.Odometry>(OdometryTopic);
        tfPublisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>(TfTopic);

        clock = new rclcs.Clock();

        odometryMsg = new nav_msgs.msg.Odometry();
        odometryMsg.header.frame_id = "/odom";
        odometryMsg.child_frame_id = "/base_footprint";
        robotPose = odometryMsg.pose;
        robotTwist = odometryMsg.twist;

        tfMsg = new tf2_msgs.msg.TFMessage();
        tfMsg.transforms_init(1);
        tfTransformStamped = tfMsg.transforms[0];
        tfTransformStamped.header.frame_id = "/odom";
        tfTransformStamped.child_frame_id = "/base_footprint";

        odometryMsg.pose.covariance = poseCovarianceDiagonal.CovarianceMatrixFromDiagonal();
        odometryMsg.twist.covariance = twistCovarianceDiagonal.CovarianceMatrixFromDiagonal();

        if (BaseRigidbody == null)
        {
            BaseRigidbody = GetComponentInChildren<Rigidbody>();
        }
    }

    new void FixedUpdate()
    {
        robotPose.pose.LocalUnity2Ros(BaseRigidbody.transform);
        robotTwist.twist.Unity2Ros(BaseRigidbody);

        odometryMsg.header.Update(clock);
        odometryPublisher.Publish(odometryMsg);

        if (PublishTf)
        {
            tfTransformStamped.header.Update(clock);
            tfTransformStamped.header.PostDate(tfPostDateMilliseconds);
            tfTransformStamped.transform.LocalUnity2Ros(BaseRigidbody.transform);
            tfPublisher.Publish(tfMsg);
        }
    }
}
