using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using rclcs;

public class ListenerExample : MonoBehaviourRosNode
{
    public string NodeName = "listener";
    public string Topic = "chatter";
    public Text ChatterText;
    protected override string nodeName { get { return NodeName; } }
    private Subscription<std_msgs.msg.String> chatterSubscription;

    protected override void StartRos()
    {
        chatterSubscription = node.CreateSubscription<std_msgs.msg.String>(
            Topic,
            (msg) => {
                Debug.Log("I heard: " + msg.Data);
                ChatterText.text = msg.Data;
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
