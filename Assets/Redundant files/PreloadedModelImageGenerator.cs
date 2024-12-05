using System.IO;
using UnityEngine;

public class PreloadedModelImageGenerator : MonoBehaviour
{
    public GameObject[] models;           // Array of all models in the scene
    public string saveFolderPath;         // Path to save the images
    public int numAngles = 10;            // Number of angles for capturing images
    public Vector3 cameraPosition = new Vector3(-0.647f, 0.253f, 0.791f); // Fixed camera position

    void Start()
    {
        // Ensure save folder exists
        if (!Directory.Exists(saveFolderPath))
        {
            Directory.CreateDirectory(saveFolderPath);
        }

        // Process each model one at a time
        for (int modelIndex = 0; modelIndex < models.Length; modelIndex++)
        {
            // Hide all models except the current one
            ShowOnlySelectedModel(models, modelIndex);

            // Capture images for the current model
            CaptureImagesForModel(models[modelIndex], modelIndex + 1);
        }

        // Show all models again after processing
        ShowAllModels(models);

        Debug.Log("Image generation completed!");
    }

    void CaptureImagesForModel(GameObject model, int modelIndex)
    {
        // Create a camera
        GameObject cameraObject = new GameObject("CaptureCamera");
        Camera camera = cameraObject.AddComponent<Camera>();

        // Set the specific camera position
        cameraObject.transform.position = cameraPosition;
        cameraObject.transform.LookAt(model.transform); // Ensure the camera points at the model

        for (int i = 0; i < numAngles; i++)
        {
            // Rotate the model around its Y-axis
            float angle = i * (360f / numAngles);
            model.transform.rotation = Quaternion.Euler(0, angle, 0);

            // Render and save the image
            SaveImage(camera, $"{saveFolderPath}/Model_{modelIndex}_Angle_{i + 1}.png");
        }

        // Reset the model's rotation after capturing images
        model.transform.rotation = Quaternion.identity;

        Destroy(cameraObject); // Clean up the camera
    }

    void SaveImage(Camera camera, string filePath)
    {
        RenderTexture renderTexture = new RenderTexture(1024, 1024, 24);
        camera.targetTexture = renderTexture;
        Texture2D texture = new Texture2D(1024, 1024, TextureFormat.RGB24, false);

        camera.Render();
        RenderTexture.active = renderTexture;
        texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture.Apply();

        byte[] bytes = texture.EncodeToPNG();
        File.WriteAllBytes(filePath, bytes);

        camera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);
        Destroy(texture);
    }

    void ShowOnlySelectedModel(GameObject[] models, int selectedIndex)
    {
        for (int i = 0; i < models.Length; i++)
        {
            if (i == selectedIndex)
            {
                models[i].SetActive(true); // Show the selected model
            }
            else
            {
                models[i].SetActive(false); // Hide all other models
            }
        }
    }

    void ShowAllModels(GameObject[] models)
    {
        foreach (var model in models)
        {
            model.SetActive(true); // Show all models
        }
    }
}
