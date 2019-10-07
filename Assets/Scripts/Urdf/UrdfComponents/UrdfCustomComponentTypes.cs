/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0
*/

using System.Collections.Generic;

namespace RosSharp.Urdf
{
    // NOTE(sam): Would make sense to use Reflection (Type.GetType) for this, but does not seem to work...
    public static class UrdfCustomComponentTypes
    {
       public static Dictionary<string, System.Type> Dictionary = new Dictionary<string, System.Type>
       {
           {"TwistBaseController", typeof(TwistBaseController) },
           {"LaserScanner2D", typeof(LaserScanner2D) },
           {"GroundTruthOdometry", typeof(GroundTruthOdometry) }
       };
    }
}
