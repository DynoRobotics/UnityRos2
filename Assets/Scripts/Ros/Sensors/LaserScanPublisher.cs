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

public class LaserScanPublisher : BehaviourNode
{
    public override string NodeName { get { return "laser_scan_publisher"; } }

    public Transform m_ScanLink;

    public float m_RangeMinimum = 0.05f;
    public float m_RangeMaximum = 25.0f;
    public float m_ApertureAngle = 270.0f;

    public int m_ScanningFrequency = 10;
    public float m_AngularResolution = 0.25f;
    public bool m_UseTimeIncrement = false;

    public LayerMask m_LayerMask = -1;
    public bool m_Visualize = true;
    public Color m_VisualizationColor = Color.red;

    public string m_ScanTopic = "/scan";
    public float m_PublisherDelay = 0.05f; // To prevent lookup into future tf2 errors

    private int m_NumLines;

    private Clock m_Clock = new Clock();
    private Publisher<sensor_msgs.msg.LaserScan> m_LaserScanPublisher;
    private Queue<sensor_msgs.msg.LaserScan> m_LaserScanMessageQueue;

    private RosTime m_LastPublishTime;

    private void Start()
    {
        if (m_ScanLink == null)
        {
            m_ScanLink = transform;
        }

        UpdateNumLines();
        m_LaserScanPublisher = node.CreatePublisher<sensor_msgs.msg.LaserScan>(m_ScanTopic);

        m_LaserScanMessageQueue = new Queue<sensor_msgs.msg.LaserScan>();

        m_LastPublishTime = m_Clock.Now;
    }

    private void UpdateNumLines()
    {
        m_NumLines = (int)Mathf.Round(m_ApertureAngle / m_AngularResolution) + 1;
    }

    private void InitLaserScanMsg(sensor_msgs.msg.LaserScan laserScanMessage)
    {
        int numLines = (int)Mathf.Round(m_ApertureAngle / m_AngularResolution) + 1;
        float timeIncrement = 0.0f;
        if (m_UseTimeIncrement)
        {
            timeIncrement = 1 / ((float)m_ScanningFrequency * (float)numLines);
        }

        laserScanMessage.header.frame_id = m_ScanLink.name;
        laserScanMessage.angle_min = Mathf.Deg2Rad * -m_ApertureAngle / 2;
        laserScanMessage.angle_max = -laserScanMessage.angle_min;
        laserScanMessage.angle_increment = Mathf.Deg2Rad * m_AngularResolution;
        laserScanMessage.time_increment = timeIncrement;
        laserScanMessage.range_max = m_RangeMaximum;
        laserScanMessage.range_min = m_RangeMinimum;

        List<float> ranges = new List<float>();
        List<float> intensities = new List<float>();

        for (int index = 0; index < m_NumLines; index++)
        {
            ranges.Add(0.0f);
            intensities.Add(0.0f);
        }

        laserScanMessage.ranges = ranges;
        laserScanMessage.intensities = intensities;
    }

    void Update()
    {
        if (m_Visualize)
        {
            for (int index = 0; index < m_NumLines; index++)
            {
                GetDistance(index, true);
            }
        }
    }

    new void FixedUpdate()
    {
        if (m_LastPublishTime.Delay(1.0f/m_ScanningFrequency).IsInThePast)
        {
            m_LastPublishTime = m_Clock.Now;
            sensor_msgs.msg.LaserScan laserScanMessage = new sensor_msgs.msg.LaserScan();
            InitLaserScanMsg(laserScanMessage);
            UptateLaserScanMessage(laserScanMessage);
            m_LaserScanMessageQueue.Enqueue(laserScanMessage);
        }

        PublishScansOlderThan(m_PublisherDelay);

        base.FixedUpdate();
    }

    private void PublishScansOlderThan(float publisherLatency)
    {
        if (m_LaserScanMessageQueue.Count > 0 && m_LaserScanMessageQueue.Peek().header.IsOlderThan(publisherLatency, m_Clock))
        {
            m_LaserScanPublisher.Publish(m_LaserScanMessageQueue.Dequeue());
        }
    }

    private void UptateLaserScanMessage(sensor_msgs.msg.LaserScan laserScanMessage)
    {
        laserScanMessage.header.Update(m_Clock);
        UpdateRanges(laserScanMessage);
    }


    private void UpdateRanges(sensor_msgs.msg.LaserScan laserScanMessage)
    {
        List<float> ranges = new List<float>();
        for (int index = 0; index < m_NumLines; index++)
        {
            ranges.Add(GetDistance(index, false));
        }

        laserScanMessage.ranges = ranges;
    }


    private float GetDistance(int index, bool visualize)
    {
        Vector3 ray = m_ScanLink.rotation * Quaternion.AngleAxis(m_ApertureAngle / 2 + (-1 * index * m_AngularResolution), Vector3.up) * Vector3.forward;

        float distance = 0.0f;
        RaycastHit hit;

        if (Physics.Raycast(m_ScanLink.position, ray, out hit, m_RangeMaximum, m_LayerMask))
        {
            distance = hit.distance;
        }

        if (visualize)
        {
            if (distance > m_RangeMinimum)
            {
                Debug.DrawRay(m_ScanLink.position, ray * distance, m_VisualizationColor);
            }
        }

        return distance;
    }
}
