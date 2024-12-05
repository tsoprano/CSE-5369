using System.Collections.Generic;
using UnityEngine;

public class GridManagerProj : MonoBehaviour
{
    public Vector3Int gridSize = new Vector3Int(15, 15, 15); // grid dimensions
    public float cellSize = 0.05f; // size of each cell
    public Vector3 gridOrigin; // Origin position of the grid
    public bool[,,] obstacleGrid; // 3D array to store obstacle info
    // public GameObject obstacle; // Cylinder obstacle reference
    public GameObject robot; // Robot reference

    [Header("Grid Drawing Settings")]
    public bool drawGrid = false; // Toggle to control grid drawing

    void Start()
    {
        // Initialize grid origin to center around the robot at (0, 0, 0)
        gridOrigin = new Vector3(-gridSize.x / 2 * cellSize, -gridSize.y / 2 * cellSize, -gridSize.z / 2 * cellSize);
        // gridOrigin = new Vector3(-0.5f, 0.0f, -0.5f);
        obstacleGrid = new bool[gridSize.x, gridSize.y, gridSize.z];
        
        // Mark both the obstacle and robot cells as occupied
        // MarkObstacleCells();
        MarkRobotCells();
    }

    // void MarkObstacleCells()
    // {
    //     // Cylinder obstacle's dimensions
    //     Vector3 obstaclePosition = obstacle.transform.position;
    //     float obstacleRadius = 0.07f;
    //     float obstacleHeight = 1.0f;
        
    //     // Calculate bounding box of the obstacle in grid space
    //     Vector3 minBounds = new Vector3(obstaclePosition.x - obstacleRadius, obstaclePosition.y - obstacleHeight / 2, obstaclePosition.z - obstacleRadius);
    //     Vector3 maxBounds = new Vector3(obstaclePosition.x + obstacleRadius, obstaclePosition.y + obstacleHeight / 2, obstaclePosition.z + obstacleRadius);

    //     // Convert bounds to grid indices
    //     Vector3Int minCell = WorldToGrid(minBounds);
    //     Vector3Int maxCell = WorldToGrid(maxBounds);

    //     // Mark the obstacle cells within the cylinder bounds
    //     for (int x = minCell.x; x <= maxCell.x; x++)
    //     {
    //         for (int y = minCell.y; y <= maxCell.y; y++)
    //         {
    //             for (int z = minCell.z; z <= maxCell.z; z++)
    //             {
    //                 if (IsWithinBounds(x, y, z))
    //                 {
    //                     Vector3 cellCenter = GridToWorld(x, y, z);
    //                     float dx = cellCenter.x - obstaclePosition.x;
    //                     float dz = cellCenter.z - obstaclePosition.z;

    //                     if (dx * dx + dz * dz <= obstacleRadius * obstacleRadius &&
    //                         cellCenter.y >= obstaclePosition.y - obstacleHeight / 2 &&
    //                         cellCenter.y <= obstaclePosition.y + obstacleHeight / 2)
    //                     {
    //                         obstacleGrid[x, y, z] = true;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    void MarkRobotCells()
    {
        ArticulationBody[] armSegments = robot.GetComponentsInChildren<ArticulationBody>();

        foreach (ArticulationBody segment in armSegments)
        {
            Collider segmentCollider = segment.GetComponent<Collider>();
            Bounds segmentBounds;

            if (segmentCollider != null)
            {
                // Use the collider bounds if available
                segmentBounds = segmentCollider.bounds;
            }
            else
            {
                // Approximate bounds by using the segment's transform position and a fixed size if no collider is found
                Vector3 segmentPosition = segment.transform.position;
                Vector3 segmentSize = new Vector3(0.1f, 0.1f, 0.1f); // Adjust the size based on typical segment size
                segmentBounds = new Bounds(segmentPosition, segmentSize);
            }

            // Calculate the bounding box of the segment in grid space
            Vector3Int minCell = WorldToGrid(segmentBounds.min);
            Vector3Int maxCell = WorldToGrid(segmentBounds.max);

            // Mark the cells within this segment's bounds
            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    for (int z = minCell.z; z <= maxCell.z; z++)
                    {
                        if (IsWithinBounds(x, y, z))
                        {
                            obstacleGrid[x, y, z] = true;
                        }
                    }
                }
            }
        }

        Debug.Log("Robot cells marked based on each arm segment.");
    }



    // Check if a grid cell is within grid bounds
    public bool IsWithinBounds(int x, int y, int z)
    {
        return x >= 0 && x < gridSize.x && y >= 0 && y < gridSize.y && z >= 0 && z < gridSize.z;
    }

    // Convert world position to grid index
    public Vector3Int WorldToGrid(Vector3 worldPos)
    {
        Vector3 localPos = worldPos - gridOrigin;
        return new Vector3Int(
            Mathf.FloorToInt(localPos.x / cellSize),
            Mathf.FloorToInt(localPos.y / cellSize),
            Mathf.FloorToInt(localPos.z / cellSize)
        );
    }

    // Convert grid index to world position
    public Vector3 GridToWorld(int x, int y, int z)
    {
        return new Vector3(
            x * cellSize + gridOrigin.x + cellSize / 2,
            y * cellSize + gridOrigin.y + cellSize / 2,
            z * cellSize + gridOrigin.z + cellSize / 2
        );
    }

    // Method to check if a cell is marked as an obstacle
    public bool IsObstacle(int x, int y, int z)
    {
        if (IsWithinBounds(x, y, z))
        {
            return obstacleGrid[x, y, z];
        }
        return false;
    }

    void OnDrawGizmos()
    {
        if (!drawGrid || obstacleGrid == null) return;

        // Loop through the grid cells
        for (int x = 0; x < gridSize.x; x++)
        {
            for (int y = 0; y < gridSize.y; y++)
            {
                for (int z = 0; z < gridSize.z; z++)
                {
                    Vector3 cellCenter = GridToWorld(x, y, z);

                    // Set color based on obstacle status
                    Gizmos.color = obstacleGrid[x, y, z] ? Color.red : Color.white;

                    // Draw the cell as a small cube
                    Gizmos.DrawWireCube(cellCenter, Vector3.one * cellSize * 0.9f);
                }
            }
        }
    }
}
