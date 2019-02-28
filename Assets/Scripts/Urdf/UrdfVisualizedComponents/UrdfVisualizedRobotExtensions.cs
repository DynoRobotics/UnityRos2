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

using UnityEditor;
using UnityEngine;
using System.IO;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfVisualizedRobotExtensions
    {
        public static void CreateFromString(string urdfString)
        {
            Robot robot = new Robot();
            robot.ConstructFromString(urdfString);

            UrdfAssetPathHandler.SetPackageRoot(Application.dataPath + "/Urdf/FromString");

            Create(robot);
        }

        public static void CreateFromFile(string filename)
        {
            Robot robot = new Robot();
            robot.ConstructFromFile(filename);

            if (!UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
            {
                Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);
                return;
            }

            UrdfAssetPathHandler.SetPackageRoot(Path.GetDirectoryName(robot.filename));

            Create(robot);
        }

        private static void Create(Robot robot)
        {
            GameObject rosVisualizerGameObject = new GameObject("RosVisualizer");
            rosVisualizerGameObject.AddComponent<TfSubscriber>();

            GameObject mapFrame = new GameObject("map");
            GameObjectUtility.SetParentAndAlign(mapFrame, rosVisualizerGameObject);
            mapFrame.AddComponent<TfFrame>();

            GameObject odomFrame = new GameObject("odom");
            GameObjectUtility.SetParentAndAlign(odomFrame, mapFrame);
            odomFrame.AddComponent<TfFrame>();

            UrdfMaterial.InitializeRobotMaterials(robot);

            UrdfVisualizedLinkExtensions.Create(odomFrame.transform, robot.root);

        }
    }
}
