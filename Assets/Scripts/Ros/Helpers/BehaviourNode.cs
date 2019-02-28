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

public abstract class BehaviourNode : MonoBehaviour
{
    public abstract string NodeName { get; }

    protected rclcs.Context context;
    protected rclcs.Node node;

    void Awake()
    {
        GameObject globalContext = GameObject.Find("Global Ros2 Context");
        if (globalContext != null)
        {
            context = globalContext.GetComponent<GlobalRosContext>().context;
        }
        else
        {
            context = new rclcs.Context();
        }

        if (!context.isInit)
        {
            rclcs.Rclcs.Init(context);
        }


        node = new rclcs.Node(NodeName, context);
    }

    protected void FixedUpdate()
    {
        //TODO(samiam): Figure out best timeout and number of callbacks to process
        for(int i = 0; i < 10; i++)
        {
            rclcs.Rclcs.SpinOnce(node, 0.0d);
        }
    }

    void OnDestroy()
    {
        node.Dispose();
        if (context.isInit)
        {
            rclcs.Rclcs.Shutdown(context);
        }
    }
}
