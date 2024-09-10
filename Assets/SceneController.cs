using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

public class SceneController : MonoBehaviour
{
    public Toggle gravityToggle;
    public Slider gravitySlider;
    public Button resetButton;
    public Button randomizeButton;
    public GameObject[] objects;

    private Vector3[] initialPositions;
    private Vector3[] initialRotations; // Store initial rotations as Euler angles

    void Start()
    {
        // Store initial positions and rotations of objects
        initialPositions = new Vector3[objects.Length];
        initialRotations = new Vector3[objects.Length];
        for (int i = 0; i < objects.Length; i++)
        {
            initialPositions[i] = objects[i].transform.position;
            initialRotations[i] = objects[i].transform.eulerAngles; // Use Euler angles instead of Quaternions
        }

        // Set up UI event listeners
        gravityToggle.onValueChanged.AddListener(ToggleGravity);
        gravitySlider.onValueChanged.AddListener(ChangeGravityStrength);
        resetButton.onClick.AddListener(ResetScene);
        randomizeButton.onClick.AddListener(RandomizeObjects);

        // Initialize gravity settings
        gravitySlider.value = 1; // Set default gravity strength
        ToggleGravity(gravityToggle.isOn);
    }

    void ToggleGravity(bool isGravityOn)
    {
        foreach (GameObject obj in objects)
        {
            obj.GetComponent<Rigidbody>().useGravity = isGravityOn;
        }
    }

    void ChangeGravityStrength(float strength)
    {
        Physics.gravity = new Vector3(0, -strength * 9.81f, 0);
    }

    void ResetScene()
    {
        for (int i = 0; i < objects.Length; i++)
        {
            objects[i].transform.position = initialPositions[i];
            objects[i].transform.eulerAngles = initialRotations[i]; // Use Euler angles for resetting rotation
            Rigidbody rb = objects[i].GetComponent<Rigidbody>();
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    void RandomizeObjects()
    {
        foreach (GameObject obj in objects)
        {
            obj.transform.position = new Vector3(Random.Range(-5, 5), Random.Range(1, 5), Random.Range(-5, 5));
            obj.transform.eulerAngles = new Vector3(Random.Range(0, 360), Random.Range(0, 360), Random.Range(0, 360)); // Randomize using Euler angles
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
