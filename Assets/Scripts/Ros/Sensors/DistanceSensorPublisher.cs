/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class DistanceSensorPublisher : BehaviourNode
{
    public string nodeName = "distance_sensor_publisher";
    public override string NodeName { get { return nodeName; } }
    public string FrameId = "distance_sensor_link";
    public string TopicName = "/distance_sensor";

    public int PublisherFrequency = 10;

    public float MinRange = 0.0f;
    public float MaxRange = 0.4f;

    public Transform SensorLink;
    public LayerMask LayerMaskDistanceSensor = -1;


    public bool Visualize = true;
    public Color VisualizationColor = Color.red;

    private Publisher<sensor_msgs.msg.Range> publisher;
    private sensor_msgs.msg.Range rangeMessage;

    private Clock clock = new Clock();
    private RosTime lastPublishTime;

    private float distance = 0.0f;
    private Vector3 ray;

    void Start()
    {
        if (SensorLink == null)
        {
            SensorLink = transform;
        }

        ray = SensorLink.TransformDirection(Vector3.forward);

        publisher = node.CreatePublisher<sensor_msgs.msg.Range>(TopicName);
        rangeMessage = new sensor_msgs.msg.Range();
        rangeMessage.header.frame_id = FrameId;
        lastPublishTime = clock.Now;
    }

    void Update()
    {
        if (Visualize && !(distance < MinRange || distance > MaxRange))
        {
            Debug.DrawRay(SensorLink.position, ray * distance, VisualizationColor);
        }
    }

    new void FixedUpdate()
    {
        if (lastPublishTime.Delay(1.0f/PublisherFrequency).IsInThePast)
        {
            lastPublishTime = clock.Now;


            RaycastHit hit;

            if (Physics.Raycast(SensorLink.position, ray, out hit, Mathf.Infinity, LayerMaskDistanceSensor))
            {
                distance = hit.distance;
            }

            rangeMessage.header.Update(clock);
            rangeMessage.range = distance;

            publisher.Publish(rangeMessage); 
        }
    }
}
