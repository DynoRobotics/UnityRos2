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
using UnityEditor;

[CustomEditor(typeof(MoveBaseGoalPublisher))]
public class MoveBaseGoalPublisherEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        MoveBaseGoalPublisher moveBaseGoalPublisher = (MoveBaseGoalPublisher)target;
        if (GUILayout.Button("Send Navigation Goal"))
        {
            moveBaseGoalPublisher.PublishNavigationGoal();
        }
    }
}
