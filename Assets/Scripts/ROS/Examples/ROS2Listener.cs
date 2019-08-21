using rclcs;
using UnityEngine;

public class ROS2Listener : MonoBehaviour
{
    private Context ctx;
    private INode node;
    private ISubscription<std_msgs.msg.String> chatter_sub;

    void Awake()
    {
        ctx = new Context();

        Rclcs.Init(ctx);

        node = Rclcs.CreateNode("UnityListener", ctx);
        chatter_sub = node.CreateSubscription<std_msgs.msg.String>(
            "/chatter", msg => Debug.Log("I heard: [" + msg.Data + "]"));
    }

    void FixedUpdate()
    {
       for(int i = 0; i < 10; i++)
        {
            Rclcs.SpinOnce(node, ctx, 0.0d);
        }
    }

    private void OnDisable()
    {
        Rclcs.Shutdown(ctx);
    }
}
