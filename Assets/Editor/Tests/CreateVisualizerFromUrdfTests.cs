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

using NUnit.Framework;
using RosSharp.Urdf.Editor;
using System.Collections;
using UnityEngine;
using UnityEngine.TestTools;

namespace Tests
{
    public class CreateVisualizerFromUrdfTests
    {
        [SetUp]
        public void BeforeEveryTest()
        {
            string urdfString =
            @"<?xml version=""1.0"" ?>
                <robot name=""test_robot"">

                    <material name=""black"">
                        <color rgba=""0.0 0.0 0.0 1.0""/>
                    </material>

                    <material name=""green"">
                        <color rgba=""0.0 0.8 0.0 1.0""/>
                    </material>

                    <link name=""base_footprint""/>

                    <joint name=""base_joint"" type=""fixed"">
                        <parent link=""base_footprint""/>
                        <child link=""base_link""/>
                        <origin xyz=""0.0 0.0 0.010"" rpy=""0 0 0""/>
                    </joint>

                    <link name=""base_link"">
                        <origin xyz=""-0.032 0 0.0"" rpy=""0 0 0.1""/>
                        <visual>
                        <origin xyz=""0.1 0.2 0.3"" rpy=""0.4 0.5 0.6""/>
                            <geometry>
                                <sphere radius=""0.1""/>
                            </geometry>
                            <material name=""light_black""/>
                        </visual>

                        <inertial>
                            <origin rpy=""0 0 0"" xyz=""0 0 -0.00207""/>
                            <mass value = ""1.0"" />
                            <inertia ixx = ""0.00499743171"" ixy = ""4.464e-08"" ixz = ""-0.00000002245"" iyy = ""0.00499741733"" iyz = ""-1.64e-09"" izz = ""0.00839239692"" />
                        </inertial >
             
                     </link>

                    <joint name=""sensor_joint"" type=""fixed"">
                        <parent link=""base_link""/>
                        <child link=""sensor_link""/>
                        <origin xyz=""0.7 0.8 0.9"" rpy=""0.1 0.2 0.3""/>
                    </joint>

                    <link name=""sensor_link"">
                        <visual>
                            <origin xyz=""0.1 0.2 0.3"" rpy=""0.4 0.5 0.6""/>
                            <geometry>
                                <sphere radius=""0.1""/>
                            </geometry>
                            <material name=""black""/>
                        </visual>

                        <inertial>
                            <origin rpy=""0 0 0"" xyz=""0 0 -0.00207""/>
                            <mass value = ""1.0"" />
                            <inertia ixx = ""0.00499743171"" ixy = ""4.464e-08"" ixz = ""-0.00000002245"" iyy = ""0.00499741733"" iyz = ""-1.64e-09"" izz = ""0.00839239692"" />
                        </inertial >

                    </link>

                    <joint name=""spinner_joint"" type=""continuous"">
                        <parent link=""base_link""/>
                        <child link=""spinner_link""/>
                        <origin xyz=""0.7 0.8 0.9"" rpy=""0.1 0.2 0.3""/>
                    </joint>

                    <link name=""spinner_link"">
                        <visual>
                            <origin xyz=""0.1 0.2 0.3"" rpy=""0.4 0.5 0.6""/>
                            <geometry>
                                <sphere radius=""0.1""/>
                            </geometry>
                            <material name=""black""/>
                        </visual>

                        <inertial>
                            <origin rpy=""0 0 0"" xyz=""0 0 -0.00207""/>
                            <mass value = ""1.0"" />
                            <inertia ixx = ""0.00499743171"" ixy = ""4.464e-08"" ixz = ""-0.00000002245"" iyy = ""0.00499741733"" iyz = ""-1.64e-09"" izz = ""0.00839239692"" />
                        </inertial >

                    </link>
                </robot>";

            UrdfVisualizedRobotExtensions.CreateFromString(urdfString);
        }

        [TearDown]
        public void AfterEveryTest()
        {
            var testGameObject = GameObject.Find("RosVisualizer");
            if (testGameObject != null)
            {
                Object.DestroyImmediate(testGameObject);
            }
        }

        [UnityTest]
        public IEnumerator _Creates_Ros_Visualizer_Game_Object()
        {
            yield return null;
            Assert.That(GameObject.Find("RosVisualizer"), Is.Not.Null);
        }

        [UnityTest]
        public IEnumerator _Creates_Map_And_Odom_Frames()
        {
            yield return null;
            GameObject rosVizGameObject = GameObject.Find("RosVisualizer");

            GameObject mapFrame = rosVizGameObject.transform.Find("map")?.gameObject;
            Assert.That(mapFrame, Is.Not.Null, "Map frame not found");

            GameObject odomFrame = mapFrame.transform.Find("odom")?.gameObject;
            Assert.That(odomFrame, Is.Not.Null, "Odom frame not found");
        }

        [UnityTest]
        public IEnumerator _Creates_Robot_Root_Frame()
        {
            yield return null;
            GameObject odomFrame = GameObject.Find("odom");
            Assert.That(odomFrame.transform.Find("base_footprint"), Is.Not.Null, 
                        "Robot root frame base_footprint not found.");
        }

        //[UnityTest]
        //public IEnumerator _Creates_Robot_Child_Frames()
        //{
        //    yield return null;
        //    GameObject baseFrame = GameObject.Find("base_link");
        //    Assert.That(baseFrame.transform.Find("spinner_link"), Is.Not.Null, 
        //        "Frame spinner_link not created as child of base_link.");
        //}
    }
}
