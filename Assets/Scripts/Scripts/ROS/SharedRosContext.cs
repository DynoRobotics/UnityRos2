using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SharedRosContext : MonoBehaviour
{
    public rclcs.Context Context;

    private void OnValidate() {
        if (Context == null)
        {
            CreateRosContext();
        }
    }

    private void Awake() {
        if (Context == null)
        {
            CreateRosContext();
        }
    }

    private void CreateRosContext()
    {
        Application.SetStackTraceLogType(LogType.Log, StackTraceLogType.Full);
        #if (UNITY_EDITOR)
        ROS2.Utils.GlobalVariables.preloadLibrary = true; 
        ROS2.Utils.GlobalVariables.preloadLibraryName = "librmw_fastrtps_cpp.so"; 
        #endif
        Context = new rclcs.Context();
        rclcs.Rclcs.Init(Context);
    }

    private void OnDestroy() {
        Debug.Log("Destrying ROS context");
        rclcs.Rclcs.Shutdown(Context);
    }
}
