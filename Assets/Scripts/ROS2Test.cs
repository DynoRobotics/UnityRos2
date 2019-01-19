using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class ROS2Test : BehaviourNode
{
    Publisher<std_msgs.msg.Bool> publisher;   

    void Start()
    {
        publisher = node.CreatePublisher<std_msgs.msg.Bool>("/publisher_topic");
        Subscription<std_msgs.msg.String> subscription = node.CreateSubscription<std_msgs.msg.String>("/test_topic", (msg) => { Debug.Log("Got something: " + msg.data); });
    }

    void Update()
    {
        std_msgs.msg.Bool msg = new std_msgs.msg.Bool
        {
            data = true
        };
        publisher.Publish(msg);
    }
}
