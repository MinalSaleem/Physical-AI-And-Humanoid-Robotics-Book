using UnityEngine;

public class UnitySimpleLidarRay : MonoBehaviour
{
    public float maxDistance = 10f;
    public LayerMask collisionLayers;

    void FixedUpdate()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, maxDistance, collisionLayers))
        {
            Debug.DrawRay(transform.position, transform.forward * hit.distance, Color.green);
            // hit.distance provides the range data
            // hit.point provides the 3D point
        }
        else
        {
            Debug.DrawRay(transform.position, transform.forward * maxDistance, Color.red);
        }
    }
}