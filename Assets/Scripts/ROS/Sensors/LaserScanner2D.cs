/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class LaserScanner2D : MonoBehaviourRosNode
{
    public string NodeName = "laser_scanner";
    public string ScanTopic = "scan";

    public Transform ScanLink;
    public string ScanLinkName;

    public float RangeMinimum = 0.05f;
    public float RangeMaximum = 25.0f;
    public float ApertureAngle = 270.0f;

    public float ScanningFrequency = 10.0f;
    public float AngularResolution = 0.25f;
    public bool UseTimeIncrement = false;

    public LayerMask LayerMask = -1;
    public bool Visualize = true;
    public Color VisualizationColor = new Color(1.0f, 0.0f, 0.0f, 0.1f);

    public float PublisherDelay = 0.1f; // To prevent lookup into future tf2 errors

    private int numLines;

    private bool shouldScan;

    protected override string nodeName { get { return NodeName; } }

    private Publisher<sensor_msgs.msg.LaserScan> scanPublisher;
    private sensor_msgs.msg.LaserScan lastSentScanMsg;
    private Queue<sensor_msgs.msg.LaserScan> scanMsgQueue;

    protected override void StartRos()
    {
        shouldScan = false;
        if (ScanLink == null)
        {
            ScanLink = transform;
        }

        if (ScanLinkName == "")
        {
            ScanLinkName = ScanLink.name;
        }

        scanMsgQueue = new Queue<sensor_msgs.msg.LaserScan>();
        lastSentScanMsg = CreateLaserScanMessage();
        scanPublisher = node.CreatePublisher<sensor_msgs.msg.LaserScan>(ScanTopic);
        StartCoroutine("TriggerScan");
        StartCoroutine("PublishScansIfOldEnough");
    }

    IEnumerator PublishScansIfOldEnough()
    {
        for (;;)
        {
            if (scanMsgQueue.Count > 0 && IsOldEnough(scanMsgQueue.Peek()))
            {
                lastSentScanMsg = scanMsgQueue.Dequeue();
                scanPublisher.Publish(lastSentScanMsg);
            }

            yield return new WaitForSeconds(0.1f / ScanningFrequency);
        }
    }

    private bool IsOldEnough(sensor_msgs.msg.LaserScan msg)
    {
        return msg.Header.IsOlderThan(PublisherDelay, clock);
    }

    private sensor_msgs.msg.LaserScan CreateLaserScanMessage()
    {
        UpdateNumLines();
        float timeIncrement = 0.0f;
        if (UseTimeIncrement)
        {
            timeIncrement = 1.0f / (ScanningFrequency * numLines);
        }

        var msg = new sensor_msgs.msg.LaserScan
        {
            Angle_min = -Mathf.Deg2Rad * ApertureAngle / 2.0f,
            Angle_max = Mathf.Deg2Rad * ApertureAngle / 2.0f,
            Angle_increment = Mathf.Deg2Rad * AngularResolution,
            Time_increment = timeIncrement,
            Range_max = RangeMaximum,
            Range_min = RangeMinimum,
            Ranges = new float[numLines],
            Intensities = new float[numLines],
        };

        msg.Header.Frame_id = ScanLinkName;

        return msg;
    }

    private void UpdateNumLines()
    {
        numLines = (int)Mathf.Round(ApertureAngle / AngularResolution) + 1;
    }

    IEnumerator TriggerScan()
    {
        for (;;)
        {
            shouldScan = true;
            yield return new WaitForSeconds(1.0f / ScanningFrequency);
        }
    }

    private void Update()
    {
        if (shouldScan)
        {
            shouldScan = false;
            scanMsgQueue.Enqueue(Scan());
        }

        if (Visualize)
        {
            VisualizeLastSentScan();
        }
    }

    private sensor_msgs.msg.LaserScan Scan()
    {
        var msg = CreateLaserScanMessage();
        msg.Header.Update(clock);

        for (int index = 0; index < numLines; index++)
        {
            var ray = CalculateRay(index);
            if (Physics.Raycast(ScanLink.position, ray, out RaycastHit hit, RangeMaximum, LayerMask))
            {
                msg.Ranges[index] = hit.distance;
            }
            else
            {
                msg.Ranges[index] = float.PositiveInfinity;
            }
        }

        return msg;
    }

    private void VisualizeLastSentScan()
    {
        for (int index = 0; index < numLines; index++)
        {
            var ray = CalculateRay(index);
            Debug.DrawRay(ScanLink.position, ray * lastSentScanMsg.Ranges[index], VisualizationColor);
        }
    }

    private Vector3 CalculateRay(int index)
    {
        return ScanLink.rotation * Quaternion.AngleAxis(ApertureAngle / 2 + (-1 * index * AngularResolution), Vector3.up) * Vector3.forward;
    }

}
