/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfSimulatedJointFactory
    {
        public static void Create(GameObject parentLinkObject, UrdfJoint joint)
        {
            JointTypes jointType = GetJointType(joint);

            if (jointType != JointTypes.Fixed)
            {
                var simulatedJoint = parentLinkObject.AddComponent<UrdfSimulatedJoint>();
                simulatedJoint.JointType = jointType;
            }
        }

        public static JointTypes GetJointType(UrdfJoint joint)
        {
            switch(joint.type)
            {
                case "fixed":
                    return JointTypes.Fixed;
                case "continuous":
                    return JointTypes.Continuous;
                case "revolute":
                    return JointTypes.Revolute;
                case "floating":
                    return JointTypes.Floating;
                case "prismatic":
                    return JointTypes.Prismatic;
                case "planar":
                    return JointTypes.Planar;
                default:
                    return JointTypes.Fixed;
            }
        }
            
    }
}
