/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public abstract class MonoBehaviourRosNode : MonoBehaviour
{
    protected abstract string nodeName { get; }
    protected Node node;
    protected int spinSomeIterations = 10;
    protected Clock clock;

    private Context context;
    private bool isAwake = false;

    private void OnValidate() {
        getSharedContext();
        if (context != null && isAwake)
        {
            StopAllCoroutines();
            CreateRosNode();
            StartRos();
        }
    }
    private void Awake() {
        StopAllCoroutines();
        CreateRosNode();
        StartRos();
        isAwake = true;
    }

    private void CreateRosNode()
    {
        getSharedContext();
        node = new Node(nodeName, context);
    }

    private void getSharedContext()
    {
        var sharedContextInstances = FindObjectsOfType(typeof(SharedRosContext));
        if (sharedContextInstances.Length > 0)
        {
            context = ((SharedRosContext)sharedContextInstances[0]).Context;
            clock = ((SharedRosContext)sharedContextInstances[0]).Clock;
        } else
        {
            Debug.LogWarning("No shared ROS context found in scene!");
        }
    }

    protected abstract void StartRos();

    protected void SpinSome()
    {
        for(int i = 0; i < spinSomeIterations; i++)
        {
            rclcs.Rclcs.SpinOnce(node, context, 0.0d);
        }
    }

    private void OnDestroy() {
        node.Dispose();
    }
}
