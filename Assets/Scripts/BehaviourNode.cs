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
        //TODO(samiam): Figure out best timeout?
        rclcs.Rclcs.SpinOnce(node, 0.001);
    }

    void OnDestroy()
    {
        node.Dispose();
        rclcs.Rclcs.Shutdown(context);
    }
}
