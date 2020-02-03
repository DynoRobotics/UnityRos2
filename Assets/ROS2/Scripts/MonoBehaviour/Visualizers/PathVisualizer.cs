/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

[RequireComponent(typeof(LineRenderer))]
public class PathVisualizer : MonoBehaviourRosNode
{
    public string NodeName = "path_visualizer";
    public string PathTopic = "plan";

    public LineRenderer PathLineRenderer;
    protected override string nodeName { get { return NodeName; } }

    Subscription<nav_msgs.msg.Path> pathSubscription;

    protected override void StartRos()
    {
        if (PathLineRenderer == null)
        {
            PathLineRenderer = GetComponent<LineRenderer>();
        }

        PathLineRenderer.positionCount = 0;

        pathSubscription = node.CreateSubscription<nav_msgs.msg.Path>(
            PathTopic,
            (msg) =>
            {
                Vector3[] unityVector3Array = msg.toUnityVector3Array();
                PathLineRenderer.positionCount = unityVector3Array.Length;
                PathLineRenderer.SetPositions(unityVector3Array);
            });
    }

    private void Update()
    {
        SpinSome();
    }
}
