using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointSpinner : BehaviourNode
{
    public string nodeName = "joint_spinner";
    public string JointStateTopic = "/joint_commands";
    public JointInterface Joint;
    public double JointVelocity = 0.1d;

    private double jointPosition = 0.0d;
    public override string NodeName { get { return nodeName; } }

    private rclcs.Publisher<sensor_msgs.msg.JointState> publisher;   
    private sensor_msgs.msg.JointState jointStateMsg = new sensor_msgs.msg.JointState();

    void Start()
    {
        publisher = node.CreatePublisher<sensor_msgs.msg.JointState>(JointStateTopic);

        if (Joint != null)
        {
            jointStateMsg.name = new List<string> { Joint.JointName };
        }
    }

    void Update()
    {
        if (Joint != null)
        {
            jointPosition += Time.deltaTime * JointVelocity;
            jointStateMsg.position = new List<double> { jointPosition };
            publisher.Publish(jointStateMsg);
        }
    }
}
