using rclcs;
using System;
using ROS2.Utils;
using UnityEngine;

public class ROS2Talker : MonoBehaviour
{
    private Context ctx;
    private INode node;
    private IPublisher<std_msgs.msg.String> chatter_pub;
    private int i = 0;

    void Awake()
    {
        ctx = new Context();

        Rclcs.Init(ctx);

        node = Rclcs.CreateNode("UnityTalker", ctx);
        chatter_pub = node.CreatePublisher<std_msgs.msg.String>("chatter");
    }

    // Update is called once per frame
    void Update()
    {
        i++;
        Debug.Log("Publishing message " + i);
        std_msgs.msg.String msg = new std_msgs.msg.String();
        if (Rclcs.Ok(ctx))
        {
            msg.Data = "Hello " + i;
            chatter_pub.Publish(msg);
        }
    }

    void OnApplicationQuit()
    {
        Debug.Log("Destroying " + i);
        Rclcs.Shutdown(ctx);
    }
}
