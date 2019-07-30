using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using rclcs;
using ROS2.Utils;

public class ListenerExample : MonoBehaviour
{
    Context ctx = new Context();
    INode node;
    ISubscription<std_msgs.msg.String> chatter_sub;
    std_msgs.msg.String msg;

    void OnValidate() {
        Start();
    }
    void Start()
    {
        Rclcs.Init(ctx);
        node = Rclcs.CreateNode("listener", ctx);

        chatter_sub = node.CreateSubscription<std_msgs.msg.String>(
            "/chatter", msg => Debug.Log("I heard: [" + msg.Data + "]"));
    }

    void FixedUpdate()
    {
        for(int i = 0; i < 10; i++)
        {
            Rclcs.SpinOnce(node, ctx, 0.0d);
        }
    }

    private void OnDisable() {
        Rclcs.Shutdown(ctx);
    }
}

