/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class TalkerExample : MonoBehaviourRosNode
{
    public string NodeName = "talker";
    public string Topic = "chatter";

    public float PublishingFrequency = 1.0f;

    protected override string nodeName { get { return NodeName; } }
    private Publisher<std_msgs.msg.String> chatterPublisher;
    private std_msgs.msg.String msg;
    private int counter;

    protected override void StartRos()
    {
        chatterPublisher = node.CreatePublisher<std_msgs.msg.String>(Topic);
        msg = new std_msgs.msg.String();
        StartCoroutine("PublishMessage");
    }

    private void Start() {
        counter = 0;
    }

    IEnumerator PublishMessage()
    {
        for (;;)
        {
            msg.Data = "Hello World" + counter.ToString();
            Debug.Log("Publishing: " + counter.ToString());
            chatterPublisher.Publish(msg);
            counter++;
            yield return new WaitForSeconds(1.0f/PublishingFrequency);
        }
    }
}