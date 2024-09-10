using UnityEngine;
using UnityEngine.UI;

// This class controls the scene's objects and UI elements (Toggle, Slider, Buttons) to manipulate object behaviors like gravity, position, and rotation.
public class SceneController : MonoBehaviour
{
    // References to the UI elements in the scene
    public Toggle gravityToggle;       // Toggle UI element to enable or disable gravity
    public Slider gravitySlider;       // Slider UI element to control gravity strength
    public Button resetButton;         // Button UI element to reset the scene
    public Button randomizeButton;     // Button UI element to randomize objects' positions and orientations
    public GameObject[] objects;       // Array of GameObjects that will be manipulated by the script

    // Arrays to store the initial positions and rotations of the objects in the scene
    private Vector3[] initialPositions;    // Array to store initial positions of each object
    private Vector3[] initialRotations;    // Array to store initial rotations (as Euler angles) of each object

    // Start is called before the first frame update
    void Start()
    {
        // Initialize arrays with the length equal to the number of objects
        initialPositions = new Vector3[objects.Length];
        initialRotations = new Vector3[objects.Length];

        // Loop through each object to store their initial positions and rotations
        for (int i = 0; i < objects.Length; i++)
        {
            // Store the initial position of the object
            initialPositions[i] = objects[i].transform.position;

            // Store the initial rotation of the object using Euler angles
            initialRotations[i] = objects[i].transform.eulerAngles;
        }

        // Add listeners to UI elements to call specific functions when their values change or are clicked
        gravityToggle.onValueChanged.AddListener(ToggleGravity);    // Calls ToggleGravity when the Toggle's value changes
        gravitySlider.onValueChanged.AddListener(ChangeGravityStrength);    // Calls ChangeGravityStrength when the Slider's value changes
        resetButton.onClick.AddListener(ResetScene);    // Calls ResetScene when the Reset button is clicked
        randomizeButton.onClick.AddListener(RandomizeObjects);    // Calls RandomizeObjects when the Randomize button is clicked

        // Initialize the gravity slider value to 1 (normal gravity)
        gravitySlider.value = 1;

        // Set the gravity state based on the Toggle's initial state (on or off)
        ToggleGravity(gravityToggle.isOn);
    }

    // Toggles the gravity on or off for all objects in the scene
    void ToggleGravity(bool isGravityOn)
    {
        // Loop through each object in the array
        foreach (GameObject obj in objects)
        {
            // Get the Rigidbody component of the object and set its 'useGravity' property based on the Toggle's state
            obj.GetComponent<Rigidbody>().useGravity = isGravityOn;
        }
    }

    // Changes the strength of gravity in the scene based on the Slider's value
    void ChangeGravityStrength(float strength)
    {
        // Set the global gravity force in Unity, scaling it based on the Slider's value (0 to 2)
        // The value is multiplied by -9.81f to represent Earth's gravity direction and scale
        Physics.gravity = new Vector3(0, -strength * 9.81f, 0);
    }

    // Resets all objects in the scene to their initial positions and rotations
    void ResetScene()
    {
        // Loop through each object in the array
        for (int i = 0; i < objects.Length; i++)
        {
            // Reset the position of the object to its stored initial position
            objects[i].transform.position = initialPositions[i];

            // Reset the rotation of the object to its stored initial rotation
            objects[i].transform.eulerAngles = initialRotations[i];

            // Get the Rigidbody component to reset velocity and angular velocity
            Rigidbody rb = objects[i].GetComponent<Rigidbody>();
            rb.velocity = Vector3.zero;          // Stop all movement
            rb.angularVelocity = Vector3.zero;   // Stop all rotation
        }
    }

    // Randomizes the positions and rotations of all objects in the scene
    void RandomizeObjects()
    {
        // Loop through each object in the array
        foreach (GameObject obj in objects)
        {
            // Set a new random position within specified ranges for x, y, and z axes
            obj.transform.position = new Vector3(
                Random.Range(-5, 5),    // Random x position between -5 and 5
                Random.Range(1, 5),     // Random y position between 1 and 5 (above the ground)
                Random.Range(-5, 5)     // Random z position between -5 and 5
            );

            // Set a new random rotation within 0 to 360 degrees for each axis
            obj.transform.eulerAngles = new Vector3(
                Random.Range(0, 360),   // Random rotation around x-axis
                Random.Range(0, 360),   // Random rotation around y-axis
                Random.Range(0, 360)    // Random rotation around z-axis
            );
        }
    }
}
