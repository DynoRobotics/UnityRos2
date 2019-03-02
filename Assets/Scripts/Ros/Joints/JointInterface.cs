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

public class JointInterface : MonoBehaviour
{
    public string JointName;

    public string JointType;

    public bool EnableJointLimits;
    public double UpperLimit;
    public double LowerLimit;

    public bool IsMoving;

    public Vector3 Axis;

    public HingeJoint rotatingJoint;
    private ConfigurableJoint prismaticJoint;

    private float jointPosition; // rad or meters

    private Quaternion originalRotation;
    private Vector3 originalPosition;

    public float Position
    {
        get
        {
            //TODO(sam): actually read position...
            return jointPosition;
        }
        set
        {
            if (EnableJointLimits)
            {
                jointPosition = Mathf.Clamp(value, (float)LowerLimit, (float)UpperLimit);
            }
            else
            {
                jointPosition = value;
            }
        }
    }

    public float Velocity
    {
        get
        {
            //TODO(sam): actually read velocity...
            return 0.0f;
            //return -rotatingJoint.velocity * Mathf.Deg2Rad; 
        }
    }

    public float Effort
    {
        get
        {
            //TODO(sam): actually read effort...
            return 0.0f;
            //return -rotatingJoint.motor.force;
        }
    }


    void Start()
    {
        rotatingJoint = GetComponent<HingeJoint>();
        originalRotation = transform.localRotation;
        originalPosition = transform.localPosition;
    }

    void Update()
    {
        //TODO(sam): use smoothing?
        if (JointType == "continuous" || JointType == "revolute")
        {
            Quaternion newLocalRotation = originalRotation * Quaternion.AngleAxis(-jointPosition * Mathf.Rad2Deg, rotatingJoint.axis);
            IsMoving = (transform.localRotation != newLocalRotation);
            transform.localRotation = newLocalRotation;
        }
        else if (JointType == "prismatic")
        {
            Vector3 newLocalPosition = originalPosition + jointPosition * Axis;
            IsMoving = (transform.localPosition != newLocalPosition);
            transform.localPosition = newLocalPosition;
        }
    }

}
