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

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfSimulatedRobotFactory
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
            GameObject robotGameObject = new GameObject(robot.name);

            UrdfMaterial.InitializeRobotMaterials(robot);

            UrdfSimulatedLinkFactory.Create(robotGameObject.transform, robot.root);

            GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
            Selection.activeObject = robotGameObject;
        }
    }
}
