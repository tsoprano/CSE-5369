using UnityEngine;
using System.Collections.Generic;

public class GDIK_PRM : MonoBehaviour
{
    public float[] jointAngles;
    public float[] jointLimitsLower = new float[] { -3.05433f, -1.5708f, -1.397485f, -3.05433f, -1.74533f, -2.57436f };
    public float[] jointLimitsUpper = new float[] { 3.05433f, 0.640187f, 1.5708f, 3.05433f, 1.91986f, 2.57436f };
    public float learningRate = 2.0f;
    public int maxIterations = 1000;
    public float positionThreshold = 0.01f;
    public ArticulationBody[] joints;
    public PRMPathfinder prmPathfinder;
    private List<Vector3> path;
    private int currentWaypointIndex = 0;
    public float[] gradients;

    void Start()
    {
        jointAngles = new float[joints.Length];
        gradients = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].xDrive.target * Mathf.Rad2Deg;
            jointLimitsLower[i] *= Mathf.Rad2Deg;
            jointLimitsUpper[i] *= Mathf.Rad2Deg;
        }
    }

    void Update()
    {
        // Wait until the path is generated in PRMPathfinding before starting to move
        if (prmPathfinder != null && prmPathfinder.isPathGenerated)
        {
            path = prmPathfinder.path;
            if (path != null && currentWaypointIndex < path.Count)
            {
                Vector3 targetPosition = path[currentWaypointIndex];
                Debug.Log($"Moving to waypoint {currentWaypointIndex} at position {targetPosition}");

                // Perform Gradient Descent IK for the current waypoint
                PerformGDIK(targetPosition);

                // Check if we have reached the waypoint
                if (Vector3.Distance(GetEndEffectorPosition(), targetPosition) < positionThreshold)
                {
                    Debug.Log($"Reached waypoint {currentWaypointIndex}");
                    currentWaypointIndex++; // Move to the next waypoint
                }
            }
            else
            {
                Debug.Log("Path completed or no path available.");
            }
        }
        else if (prmPathfinder != null && !prmPathfinder.isPathGenerated)
        {
            Debug.Log("Waiting for PRM path to be generated...");
        }
    }

    Vector3 GetEndEffectorPosition()
    {
        Matrix4x4 endEffectorTransform = CalculateForwardKinematics(jointAngles);
        return endEffectorTransform.GetColumn(3);
    }

    void PerformGDIK(Vector3 targetPosition)
    {
        for (int i = 0; i < maxIterations; i++)
        {
            Vector3 currentPosition = GetEndEffectorPosition();
            float positionError = Vector3.Distance(targetPosition, currentPosition);
            if (positionError < positionThreshold)
            {
                Debug.Log("Converged on waypoint after " + i + " iterations");
                break;
            }

            gradients = ComputeGradients(positionError, targetPosition);
            for (int j = 0; j < jointAngles.Length; j++)
            {
                jointAngles[j] -= learningRate * gradients[j];
                jointAngles[j] = Mathf.Clamp(jointAngles[j], jointLimitsLower[j], jointLimitsUpper[j]);
            }
            ApplyJointMovements(jointAngles);
        }
    }

    float[] ComputeGradients(float positionError, Vector3 targetPosition)
    {
        for (int i = 0; i < jointAngles.Length; i++)
        {
            float originalAngle = jointAngles[i];
            jointAngles[i] += 2.5f;
            Matrix4x4 perturbedTransform = CalculateForwardKinematics(jointAngles);
            Vector3 perturbedPosition = perturbedTransform.GetColumn(3);
            float positionDifference = Vector3.Distance(perturbedPosition, targetPosition);
            gradients[i] = (positionDifference - positionError) / 0.5f;
            jointAngles[i] = originalAngle;
        }
        return gradients;
    }

    void ApplyJointMovements(float[] jointAngles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = jointAngles[i];
            joints[i].xDrive = drive;
        }
    }

    public Matrix4x4 CalculateForwardKinematics(float[] joints)
    {
        Matrix4x4 T_shoulder = CreateTransformationMatrix(-Vector3.up, joints[0], new Vector3(0, 0.103f, 0));
        Matrix4x4 T_arm = CreateTransformationMatrix(-Vector3.right, joints[1], new Vector3(0, 0.08f, 0));
        Matrix4x4 T_elbow = CreateTransformationMatrix(-Vector3.right, joints[2], new Vector3(0, 0.21f, 0));
        Matrix4x4 T_forearm = CreateTransformationMatrix(Vector3.forward, joints[3], new Vector3(0, 0.03f, 0.0415f));
        Matrix4x4 T_wrist = CreateTransformationMatrix(-Vector3.up, joints[4], new Vector3(0, 0, 0.18f));
        Matrix4x4 T_hand = CreateTransformationMatrix(Vector3.up, joints[5], new Vector3(0.0164f, -0.0055f, 0));
        return T_shoulder * T_arm * T_elbow * T_forearm * T_wrist * T_hand;
    }

    public Matrix4x4 CreateTransformationMatrix(Vector3 axis, float angle, Vector3 translation)
    {
        Matrix4x4 translationMatrix = Matrix4x4.Translate(translation);
        Quaternion rotation = Quaternion.AngleAxis(angle, axis);
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        return translationMatrix * rotationMatrix;
    }
}
