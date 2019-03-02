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
using RosSharp.Urdf;

namespace Tests
{
    public class TestParseUrdf
    {
        private string urdfString;

        Robot robot;

        [SetUp]
        public void SetUp()
        {
            urdfString =
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
                                <mesh filename=""filename.stl"" scale=""0.001 0.001 0.001""/>
                            </geometry>
                        <material name=""light_black""/>
                        </visual>
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
                    </link>
                </robot>";

            robot = new Robot();
        }

        [Test]
        public void _Reads_Robot_Name()
        {
            robot.ConstructFromString(urdfString);
            Assert.That(robot.name, Is.EqualTo("test_robot"));
        }

        [Test]
        public void _Reads_Materials()
        {
            robot.ConstructFromString(urdfString);
            Assert.That(robot.materials.Count, Is.EqualTo(2));
        }

        [Test]
        public void _Reads_Links()
        {
            robot.ConstructFromString(urdfString);
            Assert.That(robot.links.Count, Is.EqualTo(3));
        }

        [Test]
        public void _Reads_Joints()
        {
            robot.ConstructFromString(urdfString);
            Assert.That(robot.joints.Count, Is.EqualTo(2));
        }

        [Test]
        public void _Reads_Material_Colors()
        {
            robot.ConstructFromString(urdfString);
            foreach (Link.Visual.Material material in robot.materials)
            {
                if (material.name == "black")
                {
                    Assert.That(material.color.rgba, Is.EqualTo(new[] { 0.0d, 0.0d, 0.0d, 1.0d }).Within(0.1));
                }
            }
        }

        [Test]
        public void _Creates_Joint_Link_Tree()
        {
            robot.ConstructFromString(urdfString);

            Link baseFootprintLink = robot.links.Find(link => link.name == "base_footprint");
            Link baseLink = robot.links.Find(link => link.name == "base_link");
            Link sensorLink = robot.links.Find(link => link.name == "sensor_link");

            Joint baseJoint = robot.joints.Find(joint => joint.name == "base_joint");
            Joint sensorJoint = robot.joints.Find(joint => joint.name == "sensor_joint");

            Assert.That(baseFootprintLink.joints.Find(joint => joint.name == "base_joint"), Is.EqualTo(baseJoint));
            Assert.That(baseLink.joints.Find(joint => joint.name == "sensor_joint"), Is.EqualTo(sensorJoint));

            Assert.That(baseJoint.ChildLink, Is.EqualTo(baseLink));
            Assert.That(sensorJoint.ChildLink, Is.EqualTo(sensorLink));
        }

        [Test]
        public void _Reads_Joint_Origins()
        {
            robot.ConstructFromString(urdfString);

            Joint sensorJoint = robot.joints.Find(joint => joint.name == "sensor_joint");
            Origin origin = sensorJoint.origin;
            Assert.That(origin.Xyz, Is.EqualTo(new[] { 0.7d, 0.8d, 0.9d }));
            Assert.That(origin.Rpy, Is.EqualTo(new[] { 0.1d, 0.2d, 0.3d }));
        }

        [Test]
        public void _Reads_Visual_Origins()
        {
            robot.ConstructFromString(urdfString);

            Link sensorLink = robot.links.Find(link => link.name == "sensor_link");
            Link.Visual visual = sensorLink.visuals[0];
            Origin origin = visual.origin;
            Assert.That(origin.Xyz, Is.EqualTo(new[] { 0.1d, 0.2d, 0.3d }));
            Assert.That(origin.Rpy, Is.EqualTo(new[] { 0.4d, 0.5d, 0.6d }));
        }

        [Test]
        public void _Reads_Link_Visual_Geometry()
        {
            robot.ConstructFromString(urdfString);

            Link sensorLink = robot.links.Find(link => link.name == "sensor_link");
            Link.Geometry geometry = sensorLink.visuals[0].geometry;
            Link.Geometry.Sphere sphere = geometry.sphere;
            Assert.That(sphere.radius, Is.EqualTo(0.1d));

            Link baseLink = robot.links.Find(link => link.name == "base_link");
            Link.Geometry.Mesh mesh = baseLink.visuals[0].geometry.mesh;
            Assert.That(mesh.filename, Is.EqualTo("filename.stl"));
            Assert.That(mesh.scale, Is.EqualTo(new[] { 0.001d, 0.001d, 0.001d }));
        }

    }
}
