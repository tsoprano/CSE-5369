using UnityEngine;
using Unity.Mathematics;
using System.Net.Sockets;
using System.Text;

public class CameraIntrinsics : MonoBehaviour 
{
    public Camera TargetCamera;
    public string PythonIPAddress = "127.0.0.1";
    public int PythonPort = 65433;
    public int width;
    public int height;

    void Start()
    {
        width = TargetCamera.pixelWidth;
        height = TargetCamera.pixelHeight;
        float3x3 intrinsics = GetIntrinsicMatrix(TargetCamera);
        SendIntrinsicToPython(intrinsics);
    }   

    private float3x3 GetIntrinsicMatrix(Camera cam)
    {
        float pixel_aspect_ratio = (float)cam.pixelWidth / (float)cam.pixelHeight;

        float alpha_u = cam.focalLength * ((float)cam.pixelWidth / cam.sensorSize.x);
        float alpha_v = cam.focalLength * pixel_aspect_ratio * ((float)cam.pixelHeight / cam.sensorSize.y);

        float u_0 = (float)cam.pixelWidth / 2;
        float v_0 = (float)cam.pixelHeight / 2;

        return new float3x3(alpha_u, 0f, u_0,
                            0f, alpha_v, v_0,
                            0f, 0f, 1f);
    }

    private void SendIntrinsicToPython(float3x3 intrinsics)
    {
        string message = $"{intrinsics.c0.x},{intrinsics.c1.y},{intrinsics.c2.x},{intrinsics.c2.y}";
        byte[] data = Encoding.UTF8.GetBytes(message);

        using (UdpClient client = new UdpClient())
        {
            client.Send(data, data.Length, PythonIPAddress, PythonPort);
        }
        Debug.Log($"Sent intrinsics to Python: {message}");
    }
}
