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

using UnityEngine;
using rclcs;

public class PubSub : BehaviourNode
{   
    public override string NodeName { get { return "joint_state_publisher"; } }

    Publisher<std_msgs.msg.Bool> publisher;   

    void Start()
    {
        publisher = node.CreatePublisher<std_msgs.msg.Bool>("/publisher_topic");
        Subscription<std_msgs.msg.String> subscription = node.CreateSubscription<std_msgs.msg.String>("/subscription_topic", (msg) => { Debug.Log("Got something: " + msg.data); });
    }

    void Update()
    {
        std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
     
        publisher.Publish(msg);
        msg.Dispose();

    }

}
