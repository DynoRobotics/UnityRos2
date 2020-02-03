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
        CreateRosContext();
        Clock = new rclcs.Clock();

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
