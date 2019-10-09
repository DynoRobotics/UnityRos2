using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class SingleTransformPublisher : MonoBehaviourRosNode
{
    public string NodeName = "single_transform_publisher";
    public float TfPublishingFrequency = 10.0f;

    public Transform ParentFrame;
    public Transform ChildFrame;

    private tf2_msgs.msg.TFMessage tfMsg;
    private Publisher<tf2_msgs.msg.TFMessage> tfPublisher;
    
    protected override string nodeName { get { return NodeName; } }

    protected override void StartRos()
    {
        if (ParentFrame == null)
        {
            ParentFrame = transform;
        }

        if (ParentFrame != null && ChildFrame != null)
        {
            tfMsg = CreateTfMsg();
            tfPublisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>("tf");

            StartCoroutine("PublishTransform");
        } else
        {
            Debug.LogWarning("Tf publisher needs both child and parent frame!");
        }

    }

    private tf2_msgs.msg.TFMessage CreateTfMsg()
    {
        var msg = new tf2_msgs.msg.TFMessage
        {
            Transforms = new geometry_msgs.msg.TransformStamped[1],
        };

        var transformStamped = new geometry_msgs.msg.TransformStamped();
        transformStamped.Header.Frame_id = ParentFrame.name;
        transformStamped.Child_frame_id = ChildFrame.name;
        msg.Transforms[0] = transformStamped;

        return msg;
    }

    IEnumerator PublishTransform()
    {
        for (;;)
        {
            tfPublisher.Publish(tfMsg);
            yield return new WaitForSeconds(1.0f / TfPublishingFrequency);
        }
    }

    private void Update()
    {
        tfMsg.Transforms[0].Header.Update(clock);
        tfMsg.Transforms[0].Transform.Unity2Ros(ChildFrame, ParentFrame);
    }
}
