using System.IO;
using UnityEngine;

public class DatasetGenerator : MonoBehaviour
{
    public Camera captureCamera; // Assign your camera here
    public GameObject[] objectsToCapture; // Assign the bottle and can objects here
    public string savePath = "D:/UnityDataset"; // Update this path as needed
    public int numAngles = 10; // Number of angles per object
    public float minDistance = 0.5f; // Minimum distance from the object
    public float maxDistance = 1.0f; // Maximum distance from the object
    public float heightOffset = 0.5f; // Height offset of the camera above the object

    private int imageIndex = 0;

    void Start()
    {
        if (!Directory.Exists(savePath))
        {
            Directory.CreateDirectory(savePath);
        }

        StartCoroutine(CaptureImages());
    }

    System.Collections.IEnumerator CaptureImages()
    {
        foreach (GameObject obj in objectsToCapture)
        {
            // Disable all objects initially
            SetAllObjectsActive(false);

            // Enable the current object
            obj.SetActive(true);

            for (int i = 0; i < numAngles; i++)
            {
                // Randomize the camera position around the object
                float angle = Random.Range(0, 360); // Random angle around the object
                float distance = Random.Range(minDistance, maxDistance); // Random distance
                float height = obj.transform.position.y + heightOffset; // Fixed height above the object

                // Calculate new camera position
                Vector3 newPosition = obj.transform.position + new Vector3(
                    Mathf.Cos(angle * Mathf.Deg2Rad) * distance,
                    height,
                    Mathf.Sin(angle * Mathf.Deg2Rad) * distance
                );

                captureCamera.transform.position = newPosition;
                captureCamera.transform.LookAt(obj.transform.position); // Make the camera look at the object

                // Wait for a frame to apply the new camera position
                yield return new WaitForEndOfFrame();

                // Capture the image
                CaptureImage(obj.name);
            }

            // Disable the current object after processing
            obj.SetActive(false);
        }

        Debug.Log("Dataset generation complete!");
    }

    void CaptureImage(string objectName)
    {
        RenderTexture renderTexture = new RenderTexture(1920, 1080, 24);
        captureCamera.targetTexture = renderTexture;

        Texture2D image = new Texture2D(1920, 1080, TextureFormat.RGB24, false);
        captureCamera.Render();

        RenderTexture.active = renderTexture;
        image.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        image.Apply();

        captureCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);

        // Save the image to disk
        byte[] bytes = image.EncodeToPNG();
        string filePath = Path.Combine(savePath, objectName, $"{imageIndex}.png");

        // Ensure directory exists
        string directoryPath = Path.GetDirectoryName(filePath);
        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        File.WriteAllBytes(filePath, bytes);
        imageIndex++;
    }

    // Helper method to disable all objects
    void SetAllObjectsActive(bool isActive)
    {
        foreach (GameObject obj in objectsToCapture)
        {
            obj.SetActive(isActive);
        }
    }
}
