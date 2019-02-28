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
    public static class UrdfSimulationLinkExtensions
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

            if (link?.inertial != null)
            {
                UrdfInertial.Create(linkObject, link.inertial);
                
                // TODO(sam): figure out if possible to only let base_footprint be kinematic
                linkObject.GetComponent<Rigidbody>().isKinematic = true;

                if (joint != null)
                {
                    UrdfSimulatedJoint.Create(linkObject, UrdfJoint.GetJointType(joint.type), joint);
                }
            }
            else if (linkObject.name == "base_footprint")
            {
                Rigidbody baseFootprintRigidbody = linkObject.AddComponent<Rigidbody>();
                baseFootprintRigidbody.useGravity = false;
                baseFootprintRigidbody.isKinematic = true;
            }
            else if (joint != null)
                Debug.LogWarning("No Joint Component will be created in GameObject \"" + linkObject.name + "\" as it has no Rigidbody Component.\n"
                                 + "Please define an Inertial for Link \"" + link.name + "\" in the URDF file to create a Rigidbody Component.\n", linkObject);

            foreach (Joint childJoint in link.joints)
            {
                Link child = childJoint.ChildLink;
                Create(linkObject.transform, child, childJoint);
            }
        }
    }
}
