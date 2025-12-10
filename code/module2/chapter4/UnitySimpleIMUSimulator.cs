using UnityEngine;

public class UnitySimpleIMUSimulator : MonoBehaviour
{
    private Vector3 previousVelocity;
    private Vector3 previousAngularVelocity;

    void FixedUpdate()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            // Linear Acceleration
            Vector3 currentVelocity = rb.velocity;
            Vector3 linearAcceleration = (currentVelocity - previousVelocity) / Time.fixedDeltaTime;
            previousVelocity = currentVelocity;

            // Angular Velocity
            // Rigidbody.angularVelocity directly provides this in rad/s
            Vector3 angularVelocity = rb.angularVelocity;

            // You can then add noise and publish this data to ROS 2
            // Debug.Log($"Linear Accel: {linearAcceleration}, Angular Vel: {angularVelocity}");
        }
    }
}