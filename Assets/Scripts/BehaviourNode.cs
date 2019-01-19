using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BehaviourNode : MonoBehaviour
{
    public string NodeName = "unity_behavoiur_node";

    rclcs.Context context;
    protected rclcs.Node node;

    void Awake()
    {
        context = new rclcs.Context();
        rclcs.Rclcs.Init(context);
        node = new rclcs.Node(NodeName, context);
    }

    void FixedUpdate()
    {
        rclcs.Rclcs.SpinOnce(node, 0.1);
    }
}
