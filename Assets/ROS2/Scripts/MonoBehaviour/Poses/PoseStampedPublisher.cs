using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class PoseStampedPublisher : MonoBehaviourRosNode
{
    public string NodeName = "pose_stamped_publisher";
    public string Topic = "pose_stamped";

    public float PublishingFrequency = 10.0f;

    public Transform parentFrame;
    public Transform childFrame;

    protected override string nodeName { get { return NodeName; } }

    private geometry_msgs.msg.PoseStamped poseMsg;
    private Publisher<geometry_msgs.msg.PoseStamped> publisher;

    protected override void StartRos()
    {
        if (parentFrame == null)
        {
            parentFrame = transform;
        }

        if (childFrame != null && parentFrame != null)
        {
            poseMsg = new geometry_msgs.msg.PoseStamped();
            poseMsg.Header.Frame_id = parentFrame.name;
            publisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>(Topic);
            StartCoroutine("PublishPose");
        }
        else
        {
            Debug.LogError("No child and/or parent frame selected!");
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
        poseMsg.Header.Update(clock);
        poseMsg.Pose.Unity2Ros(childFrame, parentFrame);
    }
}
