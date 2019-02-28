using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class DiffDriveWheelSpinner : BehaviourNode
{
    public override string NodeName { get { return "diff_drive_wheel_spinner"; } }

    public Rigidbody BaseRigidbody;
    public string JointStateTopic = "/joint_commands";

    public JointInterface LeftWheelJoint;
    public JointInterface RightWheelJoint;

    public float WheelRadius = 0.0f;
    private float WheelBase = 0.0f;

    private Publisher<sensor_msgs.msg.JointState> publisher;   
    private sensor_msgs.msg.JointState jointStateMsg = new sensor_msgs.msg.JointState();

    private double leftWheelAngularPosition = 0.0f;
    private double rightWheelAngularPosition = 0.0f;

    void Start()
    {
        publisher = node.CreatePublisher<sensor_msgs.msg.JointState>(JointStateTopic);

        if (BaseRigidbody == null)
        {
            BaseRigidbody = GetComponentInChildren<Rigidbody>();
        }

        if (LeftWheelJoint != null && RightWheelJoint != null)
        {
            //TODO(sam): figure out a better way of finding wheel radius...
            WheelRadius = LeftWheelJoint.transform.position.y - transform.position.y;
            WheelBase = Vector3.Distance(LeftWheelJoint.transform.position, RightWheelJoint.transform.position);
        }
    }

    void Update()
    {
        Vector3 localVelocity = BaseRigidbody.transform.InverseTransformDirection(BaseRigidbody.velocity).Unity2Ros();
        Vector3 localAngularVelocity = BaseRigidbody.transform.InverseTransformDirection(BaseRigidbody.angularVelocity).Unity2Ros();

        if (LeftWheelJoint != null && RightWheelJoint != null)
        {
            float leftWheelAngularVelocity = (localVelocity.x + localAngularVelocity.z * WheelBase / 2.0f) / WheelRadius;
            float rightWheelAngularVelocity = (localVelocity.x - localAngularVelocity.z * WheelBase / 2.0f) / WheelRadius;


            rightWheelAngularPosition += rightWheelAngularVelocity * Time.deltaTime;
            leftWheelAngularPosition += leftWheelAngularVelocity * Time.deltaTime;

            jointStateMsg.name = new List<string> { LeftWheelJoint.JointName, RightWheelJoint.JointName };
            jointStateMsg.position = new List<double> { leftWheelAngularPosition, rightWheelAngularPosition };

            publisher.Publish(jointStateMsg);
        }

        Rclcs.SpinOnce(node, 0.0d);
    }
}
