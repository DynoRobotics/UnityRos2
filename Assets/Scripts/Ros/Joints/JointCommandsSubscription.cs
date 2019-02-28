
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

public class JointCommandsSubscription : BehaviourNode
{
    public override string NodeName { get { return "joint_commmands_subscription"; } }
    public string TopicName = "/joint_commands";

    public Dictionary<string, JointInterface> JointInterfaceDict;
    public Dictionary<string, float> JointPositionDict;

    void Start()
    {
        JointInterfaceDict = new Dictionary<string, JointInterface>();
        JointPositionDict = new Dictionary<string, float>();
        foreach (JointInterface jointInterface in GetComponentsInChildren<JointInterface>())
        {
            JointInterfaceDict.Add(jointInterface.JointName, jointInterface);
            JointPositionDict.Add(jointInterface.JointName, 0.0F);
        }

        // Write zero position to all joints
        WritePositions();

        var subscription = node.CreateSubscription<sensor_msgs.msg.JointState>(
            TopicName, (msg) =>
            {
                List<double> msgJointPositions = msg.position;
                List<string> msgJointNames = msg.name;
                for (int i = 0; i < msg.name.Count; i++)
                {
                    JointPositionDict[msgJointNames[i]] = (float)msgJointPositions[i];
                }

                WritePositions();
            });

    }

    void WritePositions()
    {
        foreach(KeyValuePair<string, JointInterface> jointInterface in JointInterfaceDict)
        {
            jointInterface.Value.Position = JointPositionDict[jointInterface.Key];
        }
    }

}
