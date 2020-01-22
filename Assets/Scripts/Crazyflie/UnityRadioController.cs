using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using rclcs;

public class UnityRadioController : MonoBehaviourRosNode
{
    public string NodeName = "UnityRadioController";
    public string DroneId = "E7E7E7E706";
    public Text ChatterText;
    protected override string nodeName { get { return NodeName; } }
    private Subscription<geometry_msgs.msg.Pose> chatterSubscription;

    protected override void StartRos()
    {
        chatterSubscription = node.CreateSubscription<geometry_msgs.msg.Pose>(
            "/simulation/" + DroneId + "/goto",
            (msg) => {
                Debug.Log("Got pose");
                transform.localPosition = msg.Position.Ros2Unity();
            });
    }
    void Start()
    {
        
    }

    void Update()
    {
       SpinSome(); 
    }
}
