/*
© Dyno Robotics, 2019
Author: Samuel Lindgren (samuel@dynorobotics.se)
Licensed under the Apache License, Version 2.0 (the "License");

You may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class NavigationMissionPublisher : BehaviourNode
{
    public string nodeName = "mission_plan_publisher";
    public string TopicName = "/ExecuteMissionTask_command";

    public override string NodeName { get { return nodeName; } }

    public List<Transform> NavigationGoals;

    private Clock clock = new Clock();
    private Publisher<nav2_msgs.msg.MissionPlan> publisher;
    private nav2_msgs.msg.MissionPlan missionPlanMessage;


    void Start()
    {
        publisher = node.CreatePublisher<nav2_msgs.msg.MissionPlan>(TopicName);
        missionPlanMessage = new nav2_msgs.msg.MissionPlan();
    }

    public void PublishMission()
    {
        string missionPlan =
@"<root main_tree_to_execute=""MainTree"">
  <BehaviorTree ID = ""MainTree"">
    <SequenceStar name = ""root"">";

        foreach (Transform goalTransform in NavigationGoals)
        {
            Vector3 rosPosition = goalTransform.localPosition.Unity2Ros();
            string positionString =
                string.Format(@"position = ""{0};{1};{2}""",
                              rosPosition.x,
                              rosPosition.y,
                              rosPosition.z);

            Quaternion rosOrientation = goalTransform.localRotation.Unity2Ros();

            string orientationString =
                string.Format(@"orientation = ""{0};{1};{2};{3}""",
                              rosOrientation.x,
                              rosOrientation.y,
                              rosOrientation.z,
                              rosOrientation.w);


            string navigateToPoseString =
                string.Format(@"<NavigateToPose {0} {1} />", positionString, orientationString);

            missionPlan += "\n      " + navigateToPoseString;

        }

        missionPlan +=
@"
    </SequenceStar>
  </BehaviorTree>
</root >";

        Debug.Log(missionPlan);

        missionPlanMessage.mission_plan = missionPlan;

        missionPlanMessage.header.Update(clock);
        publisher.Publish(missionPlanMessage);

    }
}
