using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;

public class GDIK_Project : MonoBehaviour
{
    // ------------- Robot Configuration Parameters -----------------
    public ArticulationBody[] joints; // The robot's joints controlled via articulation bodies.

    // Gradient Descent IK Parameters
    public float[] jointAngles; // Current joint angles.
    public float[] jointLimitsLower = new float[] { -3.05433f, -1.5708f, -1.397485f, -3.05433f, -1.74533f, -2.57436f }; // Joint lower limits.
    public float[] jointLimitsUpper = new float[] { 3.05433f, 0.640187f, 1.5708f, 3.05433f, 1.91986f, 2.57436f }; // Joint upper limits.
    public float learningRate = 2.0f; // Learning rate for gradient descent.
    public int maxIterations = 1000; // Maximum number of iterations for IK.
    public float positionThreshold = 0.01f; // Threshold for position accuracy.
    private float[] gradients; // Gradient values for joint updates.

    // ------------- Object Sorting Parameters -----------------
    public GameObject redContainer; // Container for red objects.
    public GameObject yellowContainer; // Container for yellow objects.
    public GameObject[] objectPrefabs; // Prefabs for objects to be placed.
    public int numberOfObjects = 4; // Total number of objects to place.
    public int placedObjectsCount = 0; // Counter for successfully placed objects.
    public List<GameObject> generatedObjects = new List<GameObject>(); // List of generated objects.
    public Queue<Vector3> targetPositions = new Queue<Vector3>(); // Queue of object positions to pick.
    public Queue<string> targetColors = new Queue<string>(); // Queue of object colors.
    public GameObject currentObject; // Object currently being processed.
    public bool processingObject = false; // Flag to indicate if the robot is currently processing an object.

    // ------------- UDP Communication -----------------
    private UdpClient udpClient; // UDP client for receiving object positions.
    private Thread receiveThread; // Thread for listening to incoming UDP messages.
    public int port = 65432; // Port for UDP communication.

    // ------------- Robot Home Position -----------------
    private Vector3 homePosition = new Vector3(0.0f, 0.5f, 0.0f); // Home position of the robot.

    // ------------- Placement Tracking -----------------
    private List<Vector3> redUsedOffsets = new List<Vector3>(); // Track used positions in the red container.
    private List<Vector3> yellowUsedOffsets = new List<Vector3>(); // Track used positions in the yellow container.
    private bool allObjectsPlaced = false; // Flag to indicate all objects have been placed.

    void Start()
    {
        InitializeRobot(); // Initialize joint angles and limits.
        GenerateObjects(); // Generate objects to pick and place.
        StartUDPListener(); // Start listening for object positions via UDP.
    }

    void Update()
    {
        if (allObjectsPlaced)
        {
            Debug.Log("All objects placed. Robot arm at rest.");
            return; // Stop processing if all objects are placed.
        }

        // Process the next object only if no operation is ongoing.
        if (targetPositions.Count > 0 && !processingObject)
        {
            Vector3 targetPosition = targetPositions.Dequeue(); // Get the next object position.
            string color = targetColors.Dequeue(); // Get the corresponding object color.

            processingObject = true; // Indicate the robot is busy.
            StartCoroutine(PickAndPlace(targetPosition, color)); // Start pick-and-place routine.
        }
    }

    // Initialize robot parameters.
    void InitializeRobot()
    {
        jointAngles = new float[joints.Length]; // Initialize joint angles.
        gradients = new float[joints.Length]; // Initialize gradients.

        for (int i = 0; i < joints.Length; i++)
        {
            // Set initial joint angles based on xDrive target.
            jointAngles[i] = joints[i].xDrive.target * Mathf.Rad2Deg;
            // Convert joint limits from radians to degrees.
            jointLimitsLower[i] *= Mathf.Rad2Deg;
            jointLimitsUpper[i] *= Mathf.Rad2Deg;
        }
    }

    // Generate objects to be sorted.
    void GenerateObjects()
    {
        float xMin = -0.3f, xMax = 0.3f; // X-axis range for object placement.
        float zMin = 0.1f, zMax = 0.25f; // Z-axis range for object placement.
        float yLevel = 0.0f; // Y-axis level for objects.

        for (int i = 0; i < numberOfObjects; i++)
        {
            // Instantiate a random object prefab.
            GameObject obj = Instantiate(objectPrefabs[Random.Range(0, objectPrefabs.Length)]);
            obj.transform.position = new Vector3(Random.Range(xMin, xMax), yLevel, Random.Range(zMin, zMax));

            // Randomly assign red or yellow color.
            bool isRed = Random.value > 0.5f;
            Color assignedColor = isRed ? Color.red : Color.yellow;
            string assignedLabel = isRed ? "red" : "yellow";

            // Apply color and label to the object.
            obj.GetComponent<Renderer>().material.color = assignedColor;
            obj.tag = "Sortable";

            // Add ObjectLabel component for labeling.
            ObjectLabel objectLabel = obj.GetComponent<ObjectLabel>();
            if (objectLabel == null)
            {
                objectLabel = obj.AddComponent<ObjectLabel>();
            }
            objectLabel.label = assignedLabel;

            generatedObjects.Add(obj); // Add to the list of generated objects.

            Debug.Log($"Generated object: {obj.name}, Label: {objectLabel.label}, Color: {assignedColor}");
        }
    }

    // Coroutine to pick and place an object.
    System.Collections.IEnumerator PickAndPlace(Vector3 targetPosition, string queuedLabel)
    {
        // Move to the object position.
        yield return StartCoroutine(SmoothMoveToPosition(targetPosition));

        // Simulate picking the object.
        GameObject targetObject = FindClosestObject(targetPosition);
        Vector3 originalScale = Vector3.one;

        if (targetObject != null)
        {
            currentObject = targetObject; // Set the current object.

            // Save the object's original scale.
            originalScale = currentObject.transform.localScale;

            // Attach the object to the end effector.
            Transform endEffectorTransform = joints[joints.Length - 1].transform;
            currentObject.transform.SetParent(endEffectorTransform, true);
            currentObject.transform.localPosition = Vector3.zero; // Align with the end effector.
        }

        // Determine the container for the object.
        string actualLabel = queuedLabel;
        if (currentObject != null)
        {
            ObjectLabel objectLabel = currentObject.GetComponent<ObjectLabel>();
            if (objectLabel != null)
            {
                actualLabel = objectLabel.label; // Use the object's stored label.
            }
        }

        Transform container = actualLabel == "red" ? redContainer.transform : yellowContainer.transform;

        // Calculate a unique position in the container.
        Renderer containerRenderer = container.GetComponent<Renderer>();
        Vector3 containerSize = containerRenderer.bounds.size;
        float containerWidth = containerSize.x;
        float containerDepth = containerSize.z;

        List<Vector3> usedOffsets = actualLabel == "red" ? redUsedOffsets : yellowUsedOffsets;
        Vector3 offset;
        do
        {
            offset = new Vector3(
                Random.Range(-containerWidth / 2f, containerWidth / 2f),
                0f,
                Random.Range(-containerDepth / 2f, containerDepth / 2f)
            );
        } while (usedOffsets.Contains(offset)); // Avoid overlapping.

        usedOffsets.Add(offset);
        Vector3 placePosition = container.position + offset + Vector3.up * 0.1f;

        // Move to the placement position.
        yield return StartCoroutine(SmoothMoveToPosition(placePosition));

        // Simulate placing the object.
        if (currentObject != null)
        {
            currentObject.transform.SetParent(null, true); // Detach from the end effector.
            currentObject.transform.position = placePosition; // Set final position.
            currentObject.transform.localScale = originalScale; // Restore scale.

            generatedObjects.Remove(currentObject); // Remove from the list.
            currentObject = null; // Clear the current object.
            placedObjectsCount++; // Increment the counter.
        }

        // Check if all objects are placed.
        if (placedObjectsCount == numberOfObjects)
        {
            allObjectsPlaced = true;
            Debug.Log("All objects placed. Stopping robot arm.");
        }

        processingObject = false; // Mark processing as complete.
    }

    // Move smoothly to the target position.
    System.Collections.IEnumerator SmoothMoveToPosition(Vector3 targetPosition)
    {
        Vector3 startPosition = CalculateEndEffectorPosition();
        float elapsedTime = 0f;
        float duration = 2f;

        while (elapsedTime < duration)
        {
            Vector3 currentPosition = Vector3.Lerp(startPosition, targetPosition, elapsedTime / duration);
            PerformGDIK(currentPosition); // Update robot arm position.
            elapsedTime += Time.deltaTime;
            yield return null;
        }

        PerformGDIK(targetPosition); // Ensure final position is reached.
    }

    GameObject FindClosestObject(Vector3 position)
    {
        GameObject closestObject = null;
        float closestDistance = float.MaxValue;

        foreach (GameObject obj in generatedObjects)
        {
            float distance = Vector3.Distance(position, obj.transform.position);
            if (distance < closestDistance)
            {
                closestObject = obj;
                closestDistance = distance;
            }
        }

        return closestObject;
    }

    Vector3 CalculateEndEffectorPosition()
    {
        Matrix4x4 endEffectorTransform = CalculateForwardKinematics(jointAngles);
        return endEffectorTransform.GetColumn(3);
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

    void StartUDPListener()
    {
        udpClient = new UdpClient(port);
        receiveThread = new Thread(() =>
        {
            while (true)
            {
                try
                {
                    if (placedObjectsCount >= numberOfObjects)
                    {
                        continue; // Stop adding new objects if all are placed
                    }

                    IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
                    byte[] data = udpClient.Receive(ref anyIP);
                    string message = Encoding.UTF8.GetString(data);
                    string[] objects = message.Split(';');

                    foreach (string obj in objects)
                    {
                        string[] parts = obj.Split(',');
                        string color = parts[0];
                        float x = float.Parse(parts[1]);
                        float y = float.Parse(parts[2]);
                        float z = float.Parse(parts[3]);

                        Vector3 detectedPosition = new Vector3(x, y, z);

                        // Ensure the position is not already in the queue
                        if (!targetPositions.Contains(detectedPosition))
                        {
                            targetPositions.Enqueue(detectedPosition);
                            targetColors.Enqueue(color);
                        }
                    }
                }
                catch { }
            }
        });
        receiveThread.IsBackground = true;
        receiveThread.Start();
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
}
