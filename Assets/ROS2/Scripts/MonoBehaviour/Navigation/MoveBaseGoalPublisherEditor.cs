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
