/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
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

public class TfSubscriber: MonoBehaviour
{
    public string TfNamespace = "/";

    rclcs.Context context;
    protected rclcs.Node node;

    public string NodeName { get { return "unity_tf_subscriber"; } }

    private Dictionary<string, TfFrame> tfFrameDict = new Dictionary<string, TfFrame>();

    tf2_msgs.msg.TFMessage testMsg = new tf2_msgs.msg.TFMessage();

    void Awake()
    {
        context = new rclcs.Context();
        rclcs.Rclcs.Init(context);
        node = new rclcs.Node(NodeName, context);
    }

    void Start()
    {
        foreach (TfFrame tfFrame in GetComponentsInChildren<TfFrame>())
        {
            tfFrameDict.Add(TfNamespace + tfFrame.ChildFrameId, tfFrame);
        }

        QualityOfServiceProfile qos = new QualityOfServiceProfile(QosProfiles.SENSOR_DATA);

        Subscription<tf2_msgs.msg.TFMessage> tfSubscription = node.CreateSubscription<tf2_msgs.msg.TFMessage>(
            "/tf", (msg) =>
            {
                UpdateTransforms(msg);
            },
            qos);

        Subscription<tf2_msgs.msg.TFMessage> tfStaticSubscription = node.CreateSubscription<tf2_msgs.msg.TFMessage>(
            "/tf_static", (msg) =>
            {
                UpdateTransforms(msg);
            }
            );
    }

    void UpdateTransforms(tf2_msgs.msg.TFMessage msg)
    {
        foreach (var tfTransform in msg.transforms)
        {
            TfFrame tfFrame;
            tfFrameDict.TryGetValue(tfTransform.child_frame_id, out tfFrame);
            if (tfFrame != null)
            {
                tfFrame.TargetPosition = tfTransform.transform.translation.Ros2Unity();
                tfFrame.TargetRotation = tfTransform.transform.rotation.Ros2Unity();
            }
        }

    }

    void Update()
    {
        // Empty queue (depth is 5 for "SENSOR_DATA")
        for (int i = 0; i < 5; i++)
        {
            rclcs.Rclcs.SpinOnce(node, 0.0d);
        }
    }

    void OnDestroy()
    {
        node.Dispose();
        rclcs.Rclcs.Shutdown(context);
    }
}
