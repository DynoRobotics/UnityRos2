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

namespace rclcs
{

    public static class RosMessageExtensions
    {

        public static Vector3 Ros2Unity(this geometry_msgs.msg.Vector3 translation)
        {
            return new Vector3(-(float)translation.y, (float)translation.z, (float)translation.x);
        }

        public static Vector3 Ros2Unity(this geometry_msgs.msg.Point point)
        {
            return new Vector3(-(float)point.y, (float)point.z, (float)point.x);
        }

        public static Vector3 Unity2Ros(this Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }

        public static Quaternion Ros2Unity(this geometry_msgs.msg.Quaternion rotation)
        {
            return new Quaternion((float)rotation.y, -(float)rotation.z, -(float)rotation.x, (float)rotation.w);
        }

        public static Quaternion Unity2Ros(this Quaternion unityRotation)
        {
            return Quaternion.Normalize(new Quaternion(-unityRotation.z, unityRotation.x, -unityRotation.y, unityRotation.w));
        }

        public static void Unity2Ros(this geometry_msgs.msg.Pose rosPose, Transform unityTransform)
        {
            rosPose.position.Unity2Ros(unityTransform.position);
            rosPose.orientation.Unity2Ros(unityTransform.rotation);
        }

        public static void LocalUnity2Ros(this geometry_msgs.msg.Pose rosPose, Transform unityTransform)
        {
            rosPose.position.Unity2Ros(unityTransform.localPosition);
            rosPose.orientation.Unity2Ros(unityTransform.localRotation);
        }

        public static void Unity2Ros(this geometry_msgs.msg.Transform rosTransform, Transform unityTransform)
        {
            rosTransform.translation.Unity2Ros(unityTransform.position);
            rosTransform.rotation.Unity2Ros(unityTransform.rotation);
        }

        public static void LocalUnity2Ros(this geometry_msgs.msg.Transform rosTransform, Transform unityTransform)
        {
            rosTransform.translation.Unity2Ros(unityTransform.localPosition);
            rosTransform.rotation.Unity2Ros(unityTransform.localRotation);
        }

        public static void Unity2Ros(this geometry_msgs.msg.Vector3 rosVector3Msg, Vector3 unityVector3)
        {
            Vector3 rosVector3 = unityVector3.Unity2Ros();
            rosVector3Msg.x = rosVector3.x;
            rosVector3Msg.y = rosVector3.y;
            rosVector3Msg.z = rosVector3.z;
        }

        public static void Unity2Ros(this geometry_msgs.msg.Point rosPoint, Vector3 unityPosition)
        {
            Vector3 rosPosition = unityPosition.Unity2Ros();
            rosPoint.x = rosPosition.x;
            rosPoint.y = rosPosition.y;
            rosPoint.z = rosPosition.z;
        }


        public static void Unity2Ros(this geometry_msgs.msg.Quaternion rosQuaternion, Quaternion unityRotation)
        {
            Quaternion rosRotation = unityRotation.Unity2Ros(); 
            rosQuaternion.x = rosRotation.x;
            rosQuaternion.y = rosRotation.y;
            rosQuaternion.z = rosRotation.z;
            rosQuaternion.w = rosRotation.w;
        }


        public static void Unity2Ros(this geometry_msgs.msg.Twist rosTwist, Rigidbody unityRigidbody)
        {
            Vector3 rosLinearVelocity = unityRigidbody.transform.InverseTransformDirection(unityRigidbody.velocity).Unity2Ros();
            rosTwist.linear.x = rosLinearVelocity.x;
            rosTwist.linear.y = rosLinearVelocity.y;
            rosTwist.linear.z = rosLinearVelocity.z;

            Vector3 rosAngularVelocity = unityRigidbody.transform.InverseTransformDirection(unityRigidbody.angularVelocity).Unity2Ros();
            rosTwist.angular.x = rosAngularVelocity.x;
            rosTwist.angular.y = rosAngularVelocity.y;
            //TODO(sam): figure out why this is reversed
            rosTwist.angular.z = -rosAngularVelocity.z;
        }

        public static List<double> CovarianceMatrixFromDiagonal(this List<double> covarianceDiagonal)
        {
            List<double> covarianceMatrix = new List<double>();
            for(int i = 0; i < 36; i++)
            {
                covarianceMatrix.Add(0.0d);
            }

            covarianceMatrix[0] = covarianceDiagonal[0];
            covarianceMatrix[7] = covarianceDiagonal[1];
            covarianceMatrix[14] = covarianceDiagonal[2];
            covarianceMatrix[21] = covarianceDiagonal[3];
            covarianceMatrix[28] = covarianceDiagonal[4];
            covarianceMatrix[35] = covarianceDiagonal[5];

            return covarianceMatrix;
        }

        public static void Update(this std_msgs.msg.Header header, Clock clock)
        {
            var now = clock.Now;
            header.stamp.sec = now.sec;
            header.stamp.nanosec = now.nanosec;
        }

        public static void PostDate(this std_msgs.msg.Header header, uint milliSeconds)
        {
            if ((header.stamp.nanosec + (uint)1e6 * milliSeconds) > (uint)1e9)
            {
                header.stamp.sec += 1; 
                header.stamp.nanosec = header.stamp.nanosec + (uint)1e6 * milliSeconds - (uint)1e9;
            }
            else
            {
                header.stamp.nanosec += (uint)1e6 * milliSeconds;
            }
        }

        public static bool IsOlderThan(this std_msgs.msg.Header header, float seconds, Clock clock)
        {
            return header.stamp.ToRosTime(clock).Delay(seconds).IsInThePast;
        }

        public static RosTime ToRosTime(this builtin_interfaces.msg.Time timeMessage, Clock clock)
        {
            RosTime rosTime = new RosTime(clock)
            {
                sec = timeMessage.sec,
                nanosec = timeMessage.nanosec
            };

            return rosTime;
        }

        public static Vector3[] toUnityVector3Array(this nav_msgs.msg.Path pathMessage)
        {
            var rosPoses = pathMessage.poses;

            Vector3[] unityVector3Array = new Vector3[rosPoses.Count];
            for (int i = 0; i < rosPoses.Count; i++)
            {
                unityVector3Array[i] = rosPoses[i].pose.position.Ros2Unity();
            }

            return unityVector3Array;
        }
    }

}
