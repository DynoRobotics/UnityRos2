using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class PoseWithCovarianceStampedPublisher : MonoBehaviourRosNode
{
    public string NodeName = "pose_with_covariance_publisher";
    public string Topic = "pose_with_covariance_stamped";

    public float PublishingFrequency = 10.0f;

    public Transform parentFrame;
    public Transform childFrame;

    protected override string nodeName { get { return NodeName; } }

    private geometry_msgs.msg.PoseWithCovarianceStamped poseMsg;
    private Publisher<geometry_msgs.msg.PoseWithCovarianceStamped> publisher;

    protected override void StartRos()
    {
        if (parentFrame == null)
        {
            parentFrame = transform;
        }

        poseMsg = new geometry_msgs.msg.PoseWithCovarianceStamped();
        publisher = node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(Topic);

        if (childFrame == null)
        {
            Debug.LogError("No child frame selected!");
        }
        else
        {
            StartCoroutine("PublishPose");
        }
    }

    IEnumerator PublishPose()
    {
        for (;;)
        {
            publisher.Publish(poseMsg);
            yield return new WaitForSeconds(1.0f / PublishingFrequency);
        }
    }

    private void FixedUpdate()
    {
        poseMsg.Pose.Pose.Unity2Ros(childFrame, parentFrame);
    }
}
