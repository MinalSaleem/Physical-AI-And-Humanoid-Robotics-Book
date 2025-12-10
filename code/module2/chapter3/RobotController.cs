using UnityEngine;
using Unity.Robotics.ROSTCPConnector; // For ROS-Unity communication (if needed)

public class RobotController : MonoBehaviour
{
    public float moveSpeed = 1.0f;
    public float rotateSpeed = 50.0f;

    // ROSConnection ros; // Uncomment if using ROS-Unity integration

    void Start()
    {
        // ros = ROSConnection.Get = ROSConnection.GetOrCreateInstance(); // Uncomment if using ROS-Unity integration
        // ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/cmd_vel"); // Example: Register a publisher
    }

    void Update()
    {
        // Translational movement
        if (Input.GetKey(KeyCode.W))
        {
            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S))
        {
            transform.Translate(Vector3.back * moveSpeed * Time.deltaTime);
        }

        // Rotational movement
        if (Input.GetKey(KeyCode.A))
        {
            transform.Rotate(Vector3.up, -rotateSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D))
        {
            transform.Rotate(Vector3.up, rotateSpeed * Time.deltaTime);
        }

        // Example: Publish commands to ROS 2
        // RosMessageTypes.Geometry.TwistMsg twist = new RosMessageTypes.Geometry.TwistMsg(
        //     new RosMessageTypes.Geometry.Vector3Msg(Input.GetAxis("Horizontal") * moveSpeed, 0, Input.GetAxis("Vertical") * moveSpeed),
        //     new RosMessageTypes.Geometry.Vector3Msg(0, Input.GetAxis("Yaw") * rotateSpeed, 0));
        // ros.Publish("/cmd_vel", twist);
    }
}