using UnityEngine;

public class DragTarget : MonoBehaviour
{
    private Vector3 offset;
    private Plane dragPlane;
    private Camera cam;

    void Start()
    {
        cam = Camera.main; // Get the main camera
    }

    void OnMouseDown()
    {
        // Debug.Log("Mouse Down on Sphere");
        dragPlane = new Plane(cam.transform.forward, transform.position);
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);

        if (dragPlane.Raycast(ray, out float distance))
        {
            offset = transform.position - ray.GetPoint(distance);
        }
    }

    void OnMouseDrag()
    {
        // Debug.Log("Dragging Sphere");
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);

        if (dragPlane.Raycast(ray, out float distance))
        {
            transform.position = ray.GetPoint(distance) + offset;
        }
    }
}
