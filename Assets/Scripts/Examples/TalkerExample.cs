using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using rclcs;
using ROS2.Utils;

public class TalkerExample : MonoBehaviour
{
    Context ctx = new Context();
    INode node;
    IPublisher<std_msgs.msg.String> chatter_pub;
    std_msgs.msg.String msg;
    int i = 1;

    void OnValidate() {
        Start();
    }

    void Start()
    {
        Rclcs.Init(ctx);
        node = Rclcs.CreateNode("talker", ctx);

        chatter_pub = node.CreatePublisher<std_msgs.msg.String>("chatter");
        msg = new std_msgs.msg.String();
    }

    void Update()
    {
        msg.Data = "Hello World: " + i;
        i++; 
        Debug.Log("Publishing: " + msg.Data + "\"");
        chatter_pub.Publish(msg);
    }

    private void OnDisable() {
        Rclcs.Shutdown(ctx);
    }
}

