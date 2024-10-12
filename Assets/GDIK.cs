using UnityEngine;

public class GDIK : MonoBehaviour
{
    // Array to store joint angles
    public float[] jointAngles;
    
    // Limits for joint angles (from the URDF)
    public float[] jointLimitsLower = new float[] { -3.05433f, -1.5708f, -1.397485f, -3.05433f, -1.74533f, -2.57436f };
    public float[] jointLimitsUpper = new float[] { 3.05433f, 0.640187f, 1.5708f, 3.05433f, 1.91986f, 2.57436f };

    // Learning rate for gradient descent
    public float learningRate = 0.01f;

    // Maximum iterations for convergence
    public int maxIterations = 1000;

    // Threshold for stopping the gradient descent (position error)
    public float positionThreshold = 0.001f;
    public float orientationThreshold = 0.01f;

    // Reference to the draggable target (sphere) to track
    public GameObject targetDot;
    
    private float[] previousError;

    // To control joints in the Unity scene
    public ArticulationBody[] joints;

    public float[] gradients;

    void Start()
    {
        // Initialize joint angles, previous error, and gradients arrays
        jointAngles = new float[joints.Length];
        previousError = new float[joints.Length];
        gradients = new float[joints.Length];

        // Populate jointAngles with the current angles of the joints
        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].xDrive.target * Mathf.Deg2Rad;;
        }
    }

    void Update()
    {
        // Actively track the target dot
        if (targetDot != null)
        {
            // Get the position of the target dot
            Vector3 targetPosition = targetDot.transform.position;

            // Perform GDIK to follow the target
            PerformGDIK(targetPosition);
        }
    }

    void PerformGDIK(Vector3 targetPosition)
    {
        for (int i = 0; i < maxIterations; i++)
        {
            // Step 1: Compute FK (Position and Orientation)
            Matrix4x4 endEffectorTransform = CalculateForwardKinematics(jointAngles);
            Vector3 currentPosition = endEffectorTransform.GetColumn(3);

            // Step 2: Calculate position and orientation error
            float positionError = Vector3.Distance(targetPosition, currentPosition);

            // Check for convergence
            if (positionError < positionThreshold /*&& orientationError.eulerAngles.magnitude < orientationThreshold*/)
            {
                Debug.Log("Converged after " + i + " iterations");
                break;
            }

            // Step 3: Compute gradients (Jacobian)
            gradients = ComputeGradients(positionError, targetPosition);
            
            // Step 4: Update joint angles using gradient descent
            for (int j = 0; j < jointAngles.Length; j++)
            {
                jointAngles[j] -= learningRate * gradients[j];  // Gradient Descent Update
                jointAngles[j] = Mathf.Clamp(jointAngles[j], jointLimitsLower[j], jointLimitsUpper[j]);  // Respect joint limits
            }

            // Apply updated joint angles to robot
            ApplyJointMovements(jointAngles);

        }
    }

    // Function to compute gradients based on FK
   float[] ComputeGradients(float positionError, Vector3 targetPosition)
    {
        //Approximate gradient calculation using numerical differentiation
        for (int i = 0; i < jointAngles.Length; i++)
        {
            // Store the original joint angle
            float originalAngle = jointAngles[i];

            // Apply a small perturbation to the joint angle
            jointAngles[i] += 0.001f;  // Small perturbation

            // Recalculate FK with perturbed joint angle
            Matrix4x4 perturbedTransform = CalculateForwardKinematics(jointAngles);
            Vector3 perturbedPosition = perturbedTransform.GetColumn(3); 

            // Distance between perturbedPosition and targetPosition
            float positionDifference = Vector3.Distance(perturbedPosition, targetPosition);

            // Finding gradient
            gradients[i] = (positionDifference - positionError) / 0.001f;

            // Restore the original joint angle
            jointAngles[i] = originalAngle;
        }

        return gradients;
    }

    // Apply the updated joint angles to the Unity joints
    void ApplyJointMovements(float[] jointAngles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = jointAngles[i] * Mathf.Rad2Deg; // Set the joint angle
            joints[i].xDrive = drive;
        }
    }

    public Matrix4x4 CalculateForwardKinematics(float[] joints) 
    {
        // Base to joint transformations using homogeneous matrices

        Matrix4x4 T_shoulder = CreateTransformationMatrix(-Vector3.up, joints[0], new Vector3(0, 0.103f, 0)); 

        Matrix4x4 T_arm = CreateTransformationMatrix(-Vector3.right, joints[1], new Vector3(0, 0.08f, 0)); 

        Matrix4x4 T_elbow = CreateTransformationMatrix(-Vector3.right, joints[2], new Vector3(0, 0.21f, 0));

        Matrix4x4 T_forearm = CreateTransformationMatrix(Vector3.forward, joints[3], new Vector3(0, 0.03f, 0.0415f)); 

        Matrix4x4 T_wrist = CreateTransformationMatrix(-Vector3.up, joints[4], new Vector3(0, 0, 0.18f)); 

        Matrix4x4 T_hand = CreateTransformationMatrix(Vector3.up, joints[5], new Vector3(0.0164f, -0.0055f, 0)); 

        Matrix4x4 endEffectorTransform = T_shoulder * T_arm * T_elbow * T_forearm * T_wrist * T_hand;

        return endEffectorTransform; 
    }

    public Matrix4x4 CreateTransformationMatrix(Vector3 axis, float angle, Vector3 translation)
    {
        // Create a translation matrix
        Matrix4x4 translationMatrix = Matrix4x4.Translate(translation);
        
        // Create a rotation matrix based on the given axis and angle
        Quaternion rotation = Quaternion.AngleAxis(angle, axis);
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(rotation);
        
        // Return the combined transformation matrix (translation * rotation)
        return translationMatrix * rotationMatrix;
    }

}
