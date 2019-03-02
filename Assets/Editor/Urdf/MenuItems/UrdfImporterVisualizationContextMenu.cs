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

using System.IO;
using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfImporterVisualizationContextMenuItem
    {
        [MenuItem("Assets/Import Robot As Visualization From URDF")]
        private static void CreateVisualizedUrdfObject()
        {
            string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

            if (Path.GetExtension(assetPath)?.ToLower() == ".urdf")
            {
                UrdfVisualizedRobotExtensions.CreateFromFile(UrdfAssetPathHandler.GetFullAssetPath(assetPath));
            }
            else
            {
                EditorUtility.DisplayDialog("Urdf Import As Visualization",
                    "The file you selected was not a URDF file. A robot can only be imported from a valid URDF file.", "Ok");
            }
        }
    }
}
