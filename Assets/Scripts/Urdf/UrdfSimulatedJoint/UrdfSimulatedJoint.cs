/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.Urdf
{
    class UrdfSimulatedJoint
    {
        public static void Create(GameObject linkObject, UrdfJoint.JointTypes jointType, Joint joint)
        {
            Rigidbody parentRigidbody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            if (parentRigidbody == null)
            {
                Debug.LogError("No rigidbody (intertial) in joints parent link, not adding joint.");
                return;
            }

            if (jointType == UrdfJoint.JointTypes.Fixed)
            {
                UnityEngine.Joint unityJoint = linkObject.AddComponent<FixedJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;
                unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            }
            else if (jointType == UrdfJoint.JointTypes.Continuous)
            {
                UnityEngine.Joint unityJoint = linkObject.AddComponent<HingeJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;
                unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
                unityJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();
                JointInterface jointInterface = linkObject.AddComponent<JointInterface>();
                jointInterface.JointName = joint.name;
                jointInterface.JointType = "continuous";
            }
            else if (jointType == UrdfJoint.JointTypes.Revolute)
            {
                HingeJoint unityJoint = linkObject.AddComponent<HingeJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;
                unityJoint.useLimits = true;
                unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
                unityJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

                //HingeJointLimitsManager limitsManager = linkObject.AddComponent<HingeJointLimitsManager>();
                //if (joint.limit != null)
                //{
                //    limitsManager.InitializeLimits(joint.limit, unityJoint);
                //}

                JointInterface jointInterface = linkObject.AddComponent<JointInterface>();
                jointInterface.JointName = joint.name;
                jointInterface.JointType = "revolute";
                jointInterface.Axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

                if (joint.limit != null)
                {
                    jointInterface.EnableJointLimits = true;
                    jointInterface.UpperLimit = joint.limit.upper;
                    jointInterface.LowerLimit = joint.limit.lower;
                }
            }
            else if (jointType == UrdfJoint.JointTypes.Prismatic)
            {
                ConfigurableJoint unityJoint = linkObject.AddComponent<ConfigurableJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;
                unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
                unityJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

                JointInterface jointInterface = linkObject.AddComponent<JointInterface>();
                jointInterface.JointName = joint.name;
                jointInterface.JointType = "prismatic";
                jointInterface.Axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

                if (joint.limit != null)
                {
                    jointInterface.EnableJointLimits = true;
                    jointInterface.UpperLimit = joint.limit.upper;
                    jointInterface.LowerLimit = joint.limit.lower;
                }
            }
            else
            {
                Debug.LogError("Joint type not yet supported, not adding joint." + jointType);
            }
        }


        private static Vector3 GetAxis(Joint.Axis axis)
        {
            return axis.xyz.ToVector3().Ros2Unity();
        }

        protected static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }
    }
}
