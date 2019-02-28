using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

[RequireComponent(typeof(LineRenderer))]
public class PathVisualizer : BehaviourNode
{
    public string nodeName = "path_visualizer";
    public override string NodeName { get { return nodeName; } }
    public string TopicName = "/plan";

    public LineRenderer PathLineRenderer;

    private Subscription<nav_msgs.msg.Path> pathSubscription; 

    void Start()
    {
        if (PathLineRenderer == null)
        {
            PathLineRenderer = GetComponent<LineRenderer>();
        }

        PathLineRenderer.positionCount = 0;

        pathSubscription = node.CreateSubscription<nav_msgs.msg.Path>(
            TopicName, (msg) =>
            {
                Vector3[] unityVector3Array = msg.toUnityVector3Array();
                PathLineRenderer.positionCount = unityVector3Array.Length;
                PathLineRenderer.SetPositions(unityVector3Array);
            });

    }

    void Update()
    {
        
    }
}
