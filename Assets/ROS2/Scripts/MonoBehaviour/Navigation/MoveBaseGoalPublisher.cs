/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class MoveBaseGoalPublisher : MonoBehaviourRosNode
{
    public string NodeName = "move_base_goal_publisher";
    public string Topic = "/move_base_simple/goal";

    public Transform mapFrame;
    public Transform goalFrame;
    protected override string nodeName { get { return NodeName; } }

    private geometry_msgs.msg.PoseStamped goalMsg;
    private Publisher<geometry_msgs.msg.PoseStamped> goalPublisher;

    protected override void StartRos()
    {
        if (mapFrame == null)
        {
            mapFrame = transform.parent;
        }

        if (goalFrame == null)
        {
            goalFrame = transform;
        }

        goalMsg = new geometry_msgs.msg.PoseStamped();
        goalMsg.Header.Frame_id = mapFrame.name;
        goalPublisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>(Topic);
    }

    public void PublishNavigationGoal()
    {
        if (mapFrame != null && goalFrame != null)
        {
            goalMsg.Header.Update(clock);
            goalMsg.Pose.Unity2Ros(goalFrame, mapFrame);
            goalPublisher.Publish(goalMsg);
        } else
        {
            Debug.LogWarning("Map and goal frames are not both set");
        }
    }
}
