/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using UnityEngine;
using System.Xml;
using System.Xml.Linq;
using System.Collections.Generic;
using System.Linq;

namespace RosSharp.Urdf
{
    public class UrdfUnityComponent
    {
        public string reference;
        public string name;

        public UrdfUnityComponent(string reference, XElement node)
        {
            this.reference = reference;
            name = (string)node.Attribute("name");
        }
    }
}
