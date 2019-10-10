/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SharedRosContext : MonoBehaviour
{
    public rclcs.Context Context;
    public rclcs.Clock Clock;

    private void OnValidate() {
        if (Context == null)
        {
            Init();
        }
    }

    private void Awake() {
        if (Context == null)
        {
            Init();
        }
    }

    private void Init()
    {
        SetupPreload();
        CreateRosContext();
        Clock = new rclcs.Clock();

    }

    private void SetupPreload()
    {
        //Application.SetStackTraceLogType(LogType.Log, StackTraceLogType.Full);
        #if (UNITY_EDITOR)
        ROS2.Utils.GlobalVariables.preloadLibrary = true; 
        ROS2.Utils.GlobalVariables.preloadLibraryName = "librmw_fastrtps_cpp.so"; 
        #endif
    }

    private void CreateRosContext()
    {
        Context = new rclcs.Context();
        rclcs.Rclcs.Init(Context);
    }

    private void OnDestroy() {
        rclcs.Rclcs.Shutdown(Context);
    }
}
