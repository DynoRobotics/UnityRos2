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

namespace RosSharp.Urdf.Editor
{
    public static class UrdfVisualizedLinkExtensions
    {
        public static void Create(Transform parent, Link link = null, Joint joint = null)
        {
            GameObject linkObject = new GameObject("link");
            linkObject.transform.SetParentAndAlign(parent);

            UrdfVisualsExtensions.Create(linkObject.transform, link?.visuals);
            UrdfCollisionsExtensions.Create(linkObject.transform, link?.collisions);

            if (link != null)
            {
                linkObject.ImportSimulationLinkData(link, joint);
            }
            else
            {
            }
        }

        private static void ImportSimulationLinkData(this GameObject linkObject, Link link, Joint joint)
        {
            linkObject.gameObject.name = link.name;

            if (joint?.origin != null)
                UrdfOrigin.ImportOriginData(linkObject.transform, joint.origin);

            linkObject.AddComponent<TfFrame>();

            foreach (Joint childJoint in link.joints)
            {
                Link child = childJoint.ChildLink;
                Create(linkObject.transform, child, childJoint);
            }
        }
    }
}
