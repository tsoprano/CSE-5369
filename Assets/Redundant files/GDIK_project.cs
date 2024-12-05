using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class GDIK_project : MonoBehaviour
{
    // Array to store joint angles
    public float[] jointAngles;

    // Joint limits (from the URDF)
    public float[] jointLimitsLower = new float[] { -3.05433f, -1.5708f, -1.397485f, -3.05433f, -1.74533f, -2.57436f };
    public float[] jointLimitsUpper = new float[] { 3.05433f, 0.640187f, 1.5708f, 3.05433f, 1.91986f, 2.57436f };

    // Learning rate for gradient descent
    public float learningRate = 2.0f;

    // Maximum iterations for convergence
    public int maxIterations = 1000;

    // Threshold for stopping the gradient descent (position error)
    public float positionThreshold = 0.01f;

    // To control joints in the Unity scene
    public ArticulationBody[] joints;

    public float[] gradients;

    // Variables for detected object positions
    private Vector3[] objectPositions;
    private int maxObjects = 10; // Maximum number of objects to store
    private int objectCount = 0;

    // UDP communication variables
    private UdpClient udpClient;
    private Thread receiveThread;

    // Port for listening to the Python script
    public int port = 65433;

    // State variables
    private bool moveToObject = false;
    private Vector3 targetPosition;

    void Start()
    {
        // Initialize joint angles, gradients, and object positions
        jointAngles = new float[joints.Length];
        gradients = new float[joints.Length];
        objectPositions = new Vector3[maxObjects]; // Arbitrary size for storing detected objects

        // Populate jointAngles with the current angles of the joints
        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].xDrive.target * Mathf.Rad2Deg;
            jointLimitsLower[i] *= Mathf.Rad2Deg;
            jointLimitsUpper[i] *= Mathf.Rad2Deg;
        }

        // Start listening for object positions from Python
        StartUDPListener();
    }

    void Update()
    {
        // Trigger movement when objects are received and robot is not already moving
        if (!moveToObject && objectCount > 0)
        {
            // Move to a random object
            MoveToRandomObject();
        }

        // Perform GDIK if moveToObject is set
        if (moveToObject && targetPosition != Vector3.zero)
        {
            PerformGDIK(targetPosition);

            // Reset the move flag after completing the motion
            moveToObject = false;

            // Optionally return to home position after reaching the target
            // ReturnToHomePosition();
        }
    }


    // Perform gradient descent IK to move toward the target
    void PerformGDIK(Vector3 targetPosition)
    {
        for (int i = 0; i < maxIterations; i++)
        {
            // Step 1: Compute FK (Position)
            Matrix4x4 endEffectorTransform = CalculateForwardKinematics(jointAngles);
            Vector3 currentPosition = endEffectorTransform.GetColumn(3);

            // Step 2: Calculate position error
            float positionError = Vector3.Distance(targetPosition, currentPosition);

            // Check for convergence
            if (positionError < positionThreshold)
            {
                Debug.Log("Converged after " + i + " iterations");
                break;
            }

            // Step 3: Compute gradients
            gradients = ComputeGradients(positionError, targetPosition);

            // Step 4: Update joint angles using gradient descent
            for (int j = 0; j < jointAngles.Length; j++)
            {
                jointAngles[j] -= learningRate * gradients[j];
                jointAngles[j] = Mathf.Clamp(jointAngles[j], jointLimitsLower[j], jointLimitsUpper[j]);
            }

            // Apply updated joint angles to robot
            ApplyJointMovements(jointAngles);
        }
    }

    // Function to compute gradients for each joint
    float[] ComputeGradients(float positionError, Vector3 targetPosition)
    {
        for (int i = 0; i < jointAngles.Length; i++)
        {
            float originalAngle = jointAngles[i];
            jointAngles[i] += 2.5f; // Small perturbation
            Matrix4x4 perturbedTransform = CalculateForwardKinematics(jointAngles);
            Vector3 perturbedPosition = perturbedTransform.GetColumn(3);
            float positionDifference = Vector3.Distance(perturbedPosition, targetPosition);
            gradients[i] = (positionDifference - positionError) / 0.5f;
            jointAngles[i] = originalAngle; // Restore the original angle
        }

        return gradients;
    }

    // Apply joint angles to the robot
    void ApplyJointMovements(float[] jointAngles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = jointAngles[i];
            joints[i].xDrive = drive;
        }
    }

    // Calculate forward kinematics
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

    // UDP listener for receiving object positions from Python
    void StartUDPListener()
    {
        try
        {
            udpClient = new UdpClient(port);
            receiveThread = new Thread(new ThreadStart(ReceiveData));
            receiveThread.IsBackground = true;
            receiveThread.Start();
            Debug.Log("UDP listener started on port: " + port);
        }
        catch (SocketException ex)
        {
            Debug.LogError("SocketException: " + ex.Message);
        }
    }

    // Receive data from Python
    void ReceiveData()
    {
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
                byte[] data = udpClient.Receive(ref anyIP);
                string message = Encoding.UTF8.GetString(data);
                string[] objects = message.Split(';');

                // Reset object count
                objectCount = 0;

                foreach (string obj in objects)
                {
                    string[] positionData = obj.Split(',');
                    if (positionData.Length == 4)
                    {
                        float x = float.Parse(positionData[1]);
                        float y = float.Parse(positionData[2]);
                        float z = float.Parse(positionData[3]);
                        objectPositions[objectCount++] = new Vector3(x, y, z);
                    }
                }
                Debug.Log($"Received {objectCount} objects from Python.");
            }
            catch (System.Exception ex)
            {
                Debug.LogError("Error receiving UDP data: " + ex.Message);
            }
        }
    }

    private void OnApplicationQuit()
    {
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }

        if (udpClient != null)
        {
            udpClient.Close();
        }
    }

    // Trigger robot movement to a random object
    public void MoveToRandomObject()
    {
        if (objectCount > 0)
        {
            int randomIndex = Random.Range(0, objectCount);
            targetPosition = objectPositions[randomIndex];
            moveToObject = true;
            Debug.Log("Moving to object at: " + targetPosition);
        }
        else
        {
            Debug.LogWarning("No objects to move to!");
        }
    }
}
