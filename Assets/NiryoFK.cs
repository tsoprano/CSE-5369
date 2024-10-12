using UnityEngine;

public class NiryoFK : MonoBehaviour
{
    // Array to store the joint values (angles) based on the DoF of the robot
    public float[] jointValues;
    public ArticulationBody[] joints;

    // Reference to the red dot object for showing the end-effector position
    public GameObject endEffectorDot;

    // Slider for trajectory resolution
    public float trajectoryResolution = 0.1f;

    // Slider fields for joint control
    [Range(-175, 175)] public float shoulder_link = 0; 
    [Range(-90, 36.5f)] public float arm_link = 0; 
    [Range(-80, 90)] public float elbow_link = 0;
    [Range(-175, 175)] public float forearm_link = 0;
    [Range(-100, 110)] public float wrist_link = 0;
    [Range(-147.5f, 147.5f)] public float hand_link = 0;

    // Variables to show resulting position and orientation in the Inspector
    [SerializeField] private Vector3 resultingPosition;
    [SerializeField] private Vector3 resultingOrientation; // Euler angles

    
    void Update()
    {
        // 1. Assign joint angles from inspector sliders
        jointValues = new float[] { shoulder_link, arm_link, elbow_link, forearm_link, wrist_link, hand_link };
        
        // 2. Calculate forward kinematics
        Matrix4x4 endEffectorTransform = CalculateForwardKinematics();

        // 3. Extract the resulting position and orientation
        resultingPosition = ExtractPositionFromMatrix(endEffectorTransform);
        resultingOrientation = ExtractRotationFromMatrix(endEffectorTransform);

        // 4. Apply the angles to the joints in the Unity scene (move the robot)
        ApplyJointMovements();

        // 5. Update the position of the red dot (end-effector) in the scene
        if (endEffectorDot != null)
        {
            endEffectorDot.transform.position = resultingPosition; // Set red dot to the calculated position
            // endEffectorDot.transform.rotation = endEffectorTransform.rotation;

        }

    }


    public Matrix4x4 CalculateForwardKinematics() 
    {
        // Base to joint transformations using homogeneous matrices

        Matrix4x4 T_shoulder = CreateTransformationMatrix(-Vector3.up, shoulder_link, new Vector3(0, 0.103f, 0)); 

        Matrix4x4 T_arm = CreateTransformationMatrix(-Vector3.right, arm_link, new Vector3(0, 0.08f, 0)); 

        Matrix4x4 T_elbow = CreateTransformationMatrix(-Vector3.right, elbow_link, new Vector3(0, 0.21f, 0));

        Matrix4x4 T_forearm = CreateTransformationMatrix(Vector3.forward, forearm_link, new Vector3(0, 0.03f, 0.0415f));

        Matrix4x4 T_wrist = CreateTransformationMatrix(-Vector3.up, wrist_link, new Vector3(0, 0, 0.18f)); 

        Matrix4x4 T_hand = CreateTransformationMatrix(Vector3.up, hand_link, new Vector3(0.0164f, -0.0055f, 0)); 
 
        // Multiply the matrices to get the transformation from base to end-effector
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

    // Function to extract position from a 4x4 matrix
    public Vector3 ExtractPositionFromMatrix(Matrix4x4 matrix)
    {
        // return new Vector3(matrix.m30, matrix.m31, matrix.m32);
        Vector4 position = matrix.GetColumn(3);
        return new Vector3(position.x, position.y, position.z);
    }

    // Function to extract rotation from a 4x4 matrix
    public Vector3 ExtractRotationFromMatrix(Matrix4x4 matrix)
    {
        return matrix.rotation.eulerAngles;
    }

    // 4. Apply joint angles to Unity's ArticulationBodies to reflect movement in the scene
    void ApplyJointMovements()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = jointValues[i]; // Set the joint angle
            joints[i].xDrive = drive;
        }
    }
}
