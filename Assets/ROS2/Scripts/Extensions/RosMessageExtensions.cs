/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace rclcs
{

    public static class RosMessageExtensions
    {
        public static Vector3 Unity2Ros(this Vector3 unityPosition)
        {
            return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
        }
        public static Vector3 Unity2Ros(this Vector3 unityPosition, Transform referenceFrame)
        {
            return referenceFrame.InverseTransformPoint(unityPosition).Unity2Ros();
        }

        public static Vector3 Ros2Unity(this Vector3 rosPosition)
        {
            return new Vector3(-rosPosition.y, rosPosition.z, rosPosition.x);
        }

        public static geometry_msgs.msg.Vector3 Unity2RosVector3(this Vector3 unityPosition)
        {
            Vector3 rosPosition = unityPosition.Unity2Ros();
            return new geometry_msgs.msg.Vector3
            {
                X = rosPosition.x,
                Y = rosPosition.y,
                Z = rosPosition.z
            };
        }

        public static Vector3 Ros2Unity(this geometry_msgs.msg.Vector3 translation)
        {
            return new Vector3((float)translation.X, (float)translation.Y, (float)translation.Z).Ros2Unity();
        }

        public static Vector3 Ros2Unity(this geometry_msgs.msg.Point point)
        {
            return new Vector3((float)point.X, (float)point.Y, (float)point.Z).Ros2Unity();
        }

        public static Quaternion Ros2Unity(this geometry_msgs.msg.Quaternion rotation)
        {
            return new Quaternion((float)rotation.Y, -(float)rotation.Z, -(float)rotation.X, (float)rotation.W);
        }

        public static Quaternion Unity2Ros(this Quaternion unityRotation)
        {
            return Quaternion.Normalize(new Quaternion(-unityRotation.z, unityRotation.x, -unityRotation.y, unityRotation.w));
        }

        public static geometry_msgs.msg.Quaternion Unity2RosQuaternion(this Quaternion unityRotation)
        {
            var rosQuaternion = unityRotation.Unity2Ros();
            return new geometry_msgs.msg.Quaternion
            {
                X = rosQuaternion.x,
                Y = rosQuaternion.y,
                Z = rosQuaternion.z,
                W = rosQuaternion.w
            };
        }

        public static void Unity2Ros(this geometry_msgs.msg.Pose rosPose, Transform unityTransform)
        {
            rosPose.Position.Unity2Ros(unityTransform.position);
            rosPose.Orientation.Unity2Ros(unityTransform.rotation);
        }
        public static void Unity2Ros(this geometry_msgs.msg.Pose rosPose, Transform unityTransform, Transform referenceFrame)
        {
            rosPose.Position.Unity2Ros(unityTransform.position, referenceFrame);
            rosPose.Orientation.Unity2Ros(unityTransform.rotation, referenceFrame);
        }

        public static void LocalUnity2Ros(this geometry_msgs.msg.Pose rosPose, Transform unityTransform)
        {
            rosPose.Position.Unity2Ros(unityTransform.localPosition);
            rosPose.Orientation.Unity2Ros(unityTransform.localRotation);
        }

        public static void Unity2Ros(this geometry_msgs.msg.Transform rosTransform, Transform unityTransform)
        {
            rosTransform.Translation.Unity2Ros(unityTransform.position);
            rosTransform.Rotation.Unity2Ros(unityTransform.rotation);
        }

        public static void Unity2Ros(this geometry_msgs.msg.Transform rosTransform, Transform unityTransform, Transform referenceFrame)
        {
            rosTransform.Translation.Unity2Ros(referenceFrame.InverseTransformPoint(unityTransform.position));
            rosTransform.Rotation.Unity2Ros(Quaternion.Normalize(unityTransform.rotation * Quaternion.Inverse(referenceFrame.rotation)));
        }

        public static void LocalUnity2Ros(this geometry_msgs.msg.Transform rosTransform, Transform unityTransform)
        {
            rosTransform.Translation.Unity2Ros(unityTransform.localPosition);
            rosTransform.Rotation.Unity2Ros(unityTransform.localRotation);
        }

        public static void Unity2Ros(this geometry_msgs.msg.Vector3 rosVector3Msg, Vector3 unityVector3)
        {
            Vector3 rosVector3 = unityVector3.Unity2Ros();
            rosVector3Msg.X = rosVector3.x;
            rosVector3Msg.Y = rosVector3.y;
            rosVector3Msg.Z = rosVector3.z;
        }

        public static void Unity2Ros(this geometry_msgs.msg.Point rosPoint, Vector3 unityPosition)
        {
            Vector3 rosPosition = unityPosition.Unity2Ros();
            rosPoint.X = rosPosition.x;
            rosPoint.Y = rosPosition.y;
            rosPoint.Z = rosPosition.z;
        }
        public static void Unity2Ros(this geometry_msgs.msg.Point rosPoint, Vector3 unityPosition, Transform referenceFrame)
        {
            rosPoint.Unity2Ros(referenceFrame.InverseTransformPoint(unityPosition));
        }

        public static void Unity2Ros(this geometry_msgs.msg.Quaternion rosQuaternion, Quaternion unityRotation, Transform referenceFrame)
        {
            rosQuaternion.Unity2Ros(Quaternion.Normalize(unityRotation * Quaternion.Inverse(referenceFrame.rotation)));
        }
        public static void Unity2Ros(this geometry_msgs.msg.Quaternion rosQuaternion, Quaternion unityRotation)
        {
            Quaternion rosRotation = unityRotation.Unity2Ros();
            rosQuaternion.X = rosRotation.x;
            rosQuaternion.Y = rosRotation.y;
            rosQuaternion.Z = rosRotation.z;
            rosQuaternion.W = rosRotation.w;
        }

        public static List<double> CovarianceMatrixFromDiagonal(this List<double> covarianceDiagonal)
        {
            List<double> covarianceMatrix = new List<double>();
            for (int i = 0; i < 36; i++)
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
            header.Stamp.Sec = now.sec;
            header.Stamp.Nanosec = now.nanosec;
        }

        public static void PostDate(this std_msgs.msg.Header header, uint milliSeconds)
        {
            if ((header.Stamp.Nanosec + (uint)1e6 * milliSeconds) > (uint)1e9)
            {
                header.Stamp.Sec += 1;
                header.Stamp.Nanosec = header.Stamp.Nanosec + (uint)1e6 * milliSeconds - (uint)1e9;
            }
            else
            {
                header.Stamp.Nanosec += (uint)1e6 * milliSeconds;
            }
        }

        public static bool IsOlderThan(this std_msgs.msg.Header header, float seconds, Clock clock)
        {
            return header.Stamp.ToRosTime(clock).Delay(seconds).IsInThePast;
        }

        public static RosTime ToRosTime(this builtin_interfaces.msg.Time timeMessage, Clock clock)
        {
            RosTime rosTime = new RosTime(clock)
            {
                sec = timeMessage.Sec,
                nanosec = timeMessage.Nanosec
            };

            return rosTime;
        }

        public static void Unity2Ros(this geometry_msgs.msg.Twist rosTwist, Rigidbody unityRigidbody)
        {
            Vector3 rosLinearVelocity = unityRigidbody.transform.InverseTransformDirection(unityRigidbody.velocity).Unity2Ros();
            rosTwist.Linear.X = rosLinearVelocity.x;
            rosTwist.Linear.Y = rosLinearVelocity.y;
            rosTwist.Linear.Z = rosLinearVelocity.z;

            Vector3 rosAngularVelocity = unityRigidbody.transform.InverseTransformDirection(unityRigidbody.angularVelocity).Unity2Ros();
            rosTwist.Angular.X = rosAngularVelocity.x;
            rosTwist.Angular.Y = rosAngularVelocity.y;
            //TODO(sam): figure out why this is reversed
            rosTwist.Angular.Z = -rosAngularVelocity.z;
        }

        public static Vector3[] toUnityVector3Array(this nav_msgs.msg.Path pathMessage)
        {
            var rosPoses = pathMessage.Poses;

            Vector3[] unityVector3Array = new Vector3[rosPoses.Length];
            for (int i = 0; i < rosPoses.Length; i++)
            {
                unityVector3Array[i] = rosPoses[i].Pose.Position.Ros2Unity();
            }

            return unityVector3Array;
        }
    }

}

