/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class GroundTruthOdometry : MonoBehaviourRosNode
{
    public string NodeName = "odometry";
    public string OdometryTopic = "odom";

    public float OdometryPublishingFrequency = 10.0f;
    public float TfPublishingFrequency = 10.0f;

    public Rigidbody BaseRigidbody;

    public Transform OdomReferanceFrame;

    public bool PublishTf = true;
    protected override string nodeName { get { return NodeName; } }

    private nav_msgs.msg.Odometry odometryMsg;
    private Publisher<nav_msgs.msg.Odometry> odometryPublisher;

    private tf2_msgs.msg.TFMessage tfMsg;
    private Publisher<tf2_msgs.msg.TFMessage> tfPublisher;

    //private List<double> poseCovarianceDiagonal = new List<double> { 0.001d, 0.001d, 0.001d, 0.001d, 0.001d, 0.03d };
    //private List<double> twistCovarianceDiagonal = new List<double> { 0.001d, 0.001d, 0.001d, 0.001d, 0.001d, 0.03d };

    protected override void StartRos()
    {
        odometryMsg = CreateOdometryMsg();
        odometryPublisher = node.CreatePublisher<nav_msgs.msg.Odometry>(OdometryTopic);

        tfMsg = CreateTfMsg();
        tfPublisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>("tf");

        if (BaseRigidbody == null)
        {
            BaseRigidbody = GetComponent<Rigidbody>();
        }

        StartCoroutine("PublishOdometry");
        if (PublishTf)
        {
            StartCoroutine("PublishTfRoutine");
        }
    }

    IEnumerator PublishOdometry()
    {
        for (;;)
        {
            odometryPublisher.Publish(odometryMsg);
            yield return new WaitForSeconds(1.0f / OdometryPublishingFrequency);
        }
    }
    IEnumerator PublishTfRoutine()
    {
        for (;;)
        {
            tfPublisher.Publish(tfMsg);
            yield return new WaitForSeconds(1.0f / TfPublishingFrequency);
        }
    }

    private nav_msgs.msg.Odometry CreateOdometryMsg()
    {
        var msg = new nav_msgs.msg.Odometry();

        msg.Child_frame_id = "/base_footprint";
        msg.Header.Frame_id = "/odom";

        // TODO(sam): Add support for fixed size arrays to rcl_dotnet
        // msg.Pose.Covariance = poseCovarianceDiagonal.CovarianceMatrixFromDiagonal();
        // msg.Twist.Covariance = twistCovarianceDiagonal.CovarianceMatrixFromDiagonal();

        return msg;
    }

    private tf2_msgs.msg.TFMessage CreateTfMsg()
    {
        var msg = new tf2_msgs.msg.TFMessage
        {
            Transforms = new geometry_msgs.msg.TransformStamped[1],
        };

        var transformStamped = new geometry_msgs.msg.TransformStamped();
        transformStamped.Header.Frame_id = "/odom";
        transformStamped.Child_frame_id = "/base_footprint";
        msg.Transforms[0] = transformStamped;

        return msg;
    }

    private void FixedUpdate()
    {
        // TODO(sam): Add (optional) drift and noise

        odometryMsg.Header.Update(clock);
        if (OdomReferanceFrame == null)
        {
            odometryMsg.Pose.Pose.LocalUnity2Ros(BaseRigidbody.transform);
        } else
        {
            odometryMsg.Pose.Pose.Unity2Ros(BaseRigidbody.transform, OdomReferanceFrame);
        }

        odometryMsg.Twist.Twist.Unity2Ros(BaseRigidbody);

        if (PublishTf)
        {
            tfMsg.Transforms[0].Header.Update(clock);
            if (OdomReferanceFrame == null)
            {
                tfMsg.Transforms[0].Transform.LocalUnity2Ros(BaseRigidbody.transform);
            }
            else
            {
                tfMsg.Transforms[0].Transform.Unity2Ros(BaseRigidbody.transform, OdomReferanceFrame);
            }
        }
    }

}
