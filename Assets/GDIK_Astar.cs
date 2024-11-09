using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using System.Linq;

public class GDIK_Astar : MonoBehaviour
{
    // Array to store joint angles
    public float[] jointAngles;
    
    // Limits for joint angles (from the URDF)
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

    // Variables for object detection data
    private Vector3 objectPosition;
    private List<Vector3> plannedPath = null;

    // Obstacle reference
    public Transform wall;

    // UDP communication variables
    private UdpClient udpClient;
    private Thread receiveThread;

    // Port for listening to the Python script
    public int port = 65433;

    void Start()
    {
        // Initialize joint angles, previous error, and gradients arrays
        jointAngles = new float[joints.Length];
        gradients = new float[joints.Length];

        // Populate jointAngles with the current angles of the joints
        for (int i = 0; i < joints.Length; i++)
        {
            jointAngles[i] = joints[i].xDrive.target * Mathf.Rad2Deg;
            jointLimitsLower[i] *= Mathf.Rad2Deg;
            jointLimitsUpper[i] *= Mathf.Rad2Deg;
        }

        // Default object position
        objectPosition = new Vector3(0.5f, 0.0f, 0.5f);

        // Start listening for the object position from Python
        StartUDPListener();
    }

    void Update()
    {
        // // Path planning
        // if (objectPosition != Vector3.zero)
        // {
        //     plannedPath = PlanPathAStar(transform.position, objectPosition);
        // }

        // // Follow the path
        // if (plannedPath != null && plannedPath.Count > 0)
        // {
        //     Vector3 nextPoint = plannedPath[0];
        //     transform.position = Vector3.MoveTowards(transform.position, nextPoint, Time.deltaTime);
        //     if (transform.position == nextPoint)
        //     {
        //         plannedPath.RemoveAt(0);
        //     }
        // }

        // Check if object position has been set by Python
        if (objectPosition != Vector3.zero)
        {
            PerformGDIK(objectPosition);
        }
    }

    // Function to update the object position when received from Python
    public void UpdateTargetPosition(Vector3 newPosition)
    {
        objectPosition = newPosition;
        Debug.Log("Updated target position: " + newPosition);
    }

    void PerformGDIK(Vector3 targetPosition)
    {
        for (int i = 0; i < maxIterations; i++)
        {
            // Step 1: Compute FK (Position and Orientation)
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
        for (int i = 0; i < jointAngles.Length; i++)
        {
            float originalAngle = jointAngles[i];

            jointAngles[i] += 2.5f;  // Small perturbation
            Matrix4x4 perturbedTransform = CalculateForwardKinematics(jointAngles);
            Vector3 perturbedPosition = perturbedTransform.GetColumn(3);

            float positionDifference = Vector3.Distance(perturbedPosition, targetPosition);
            gradients[i] = (positionDifference - positionError) / 0.5f;

            jointAngles[i] = originalAngle;  // Restore the original joint angle
        }

        return gradients;
    }

    void ApplyJointMovements(float[] jointAngles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = jointAngles[i];  // Set the joint angle
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

    private List<Vector3> PlanPathAStar(Vector3 start, Vector3 target)
    {
        Debug.Log("=========PlanPathAStar===========");
        var openList = new List<Vector3> { start };
        var closedList = new HashSet<Vector3>();
        var cameFrom = new Dictionary<Vector3, Vector3>();
        var gScore = new Dictionary<Vector3, float> { [start] = 0 };
        var fScore = new Dictionary<Vector3, float> { [start] = Heuristic(start, target) };

        while (openList.Count > 0)
        {
            Vector3 current = openList.OrderBy(n => fScore.ContainsKey(n) ? fScore[n] : Mathf.Infinity).First();

            if (current == target)
                return ReconstructPath(cameFrom, current);

            openList.Remove(current);
            closedList.Add(current);

            foreach (Vector3 neighbor in GetNeighbors(current))
            {
                if (closedList.Contains(neighbor) || IsCollision(neighbor)) continue;

                float tentativeG = gScore[current] + Vector3.Distance(current, neighbor);

                if (!openList.Contains(neighbor)) openList.Add(neighbor);
                else if (tentativeG >= gScore[neighbor]) continue;

                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, target);
            }
        }
        return null;
    }

    private List<Vector3> ReconstructPath(Dictionary<Vector3, Vector3> cameFrom, Vector3 current)
    {
        var totalPath = new List<Vector3> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            totalPath.Insert(0, current);
        }
        return totalPath;
    }

    private List<Vector3> GetNeighbors(Vector3 node)
    {
        return new List<Vector3>
        {
            node + new Vector3(1, 0, 0),
            node + new Vector3(-1, 0, 0),
            node + new Vector3(0, 1, 0),
            node + new Vector3(0, -1, 0),
            node + new Vector3(0, 0, 1),
            node + new Vector3(0, 0, -1),
        };
    }

    private bool IsCollision(Vector3 position)
    {
        return Physics.CheckBox(position, wall.localScale / 2, wall.rotation, LayerMask.GetMask("Obstacle"));
    }

    private float Heuristic(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b);
    }

    void StartUDPListener()
    {
        try
        {
            Debug.Log("Starting UDP listener...");
            udpClient = new UdpClient(port);
            Debug.Log("UDP listener started on port: " + port);
            receiveThread = new Thread(new ThreadStart(ReceiveData));
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }
        catch (SocketException ex)
        {
            Debug.LogError("SocketException: " + ex.Message);
        }
    }

    void ReceiveData()
    {
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
                byte[] data = udpClient.Receive(ref anyIP);
                string message = Encoding.UTF8.GetString(data);
                string[] positionData = message.Split(',');

                if (positionData.Length == 3)
                {
                    float x = float.Parse(positionData[0]);
                    float y = float.Parse(positionData[1]);
                    float z = float.Parse(positionData[2]);

                    Vector3 newPosition = new Vector3(x, y, z);
                    UpdateTargetPosition(newPosition);
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError("Error receiving UDP data: " + ex.Message);
            }
        }
    }

    private void OnApplicationQuit()
    {
        // Stop the UDP listener when the application quits
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }

        if (udpClient != null)
        {
            udpClient.Close();
        }
    }
}
