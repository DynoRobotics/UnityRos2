using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public abstract class MonoBehaviourRosNode : MonoBehaviour
{
    protected abstract string nodeName { get; }
    protected Node node;

    protected int spinSomeIterations = 10;

    private Context context;

    private void OnValidate() {
        getContext();
        if (context != null)
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
    }

    private void CreateRosNode()
    {
        getContext();
        node = new Node(nodeName, context);
    }

    private void getContext()
    {
        var sharedContextInstances = FindObjectsOfType(typeof(SharedRosContext));
        if (sharedContextInstances.Length > 0)
        {
            context = ((SharedRosContext)sharedContextInstances[0]).Context;
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
