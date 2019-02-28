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

public class TfFrame : MonoBehaviour
{
    //public Vector3 TargetPosition { set { targetPosition = value; } }
    //public Quaternion TargetRotation { set { targetRotation = value; } }

    public Vector3 TargetPosition { set { transform.localPosition = value; } }
    public Quaternion TargetRotation { set { transform.localRotation = value; } }

    public string ChildFrameId;

    private Vector3 targetPosition;
    private Quaternion targetRotation;


    void Awake()
    {
        ChildFrameId = name;
    }

    void FixedUpdate()
    {
        //if (targetPosition != null && targetRotation != null)
        //{
        //    //TODO(sam): add smoothing?
        //    transform.localPosition = targetPosition;
        //    transform.localRotation = targetRotation;
        //}
    }
}