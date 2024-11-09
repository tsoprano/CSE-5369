using System.Collections.Generic;
using UnityEngine;

public class PRMPathfinder : MonoBehaviour
{
    public int sampleCount = 50; // Number of nodes to sample for PRM
    public float connectionRadius = 0.5f; // Radius within which nodes can be connected
    public GridManager gridManager; // Reference to GridManager
    public GameObject endEffector; // End effector GameObject (robot's current position)
    public GameObject goalObject; // Goal position GameObject
    private List<Vector3> sampledNodes;
    public List<Vector3> path;
    public bool isPathGenerated = false; // Flag to indicate path generation

    [Header("Path and sample points draw Settings")]
    public bool drawPath = false; // Toggle to control grid drawing


    void Start()
    {
        if (!gridManager)
        {
            Debug.LogError("GridManager not assigned!");
            return;
        }

        if (!endEffector)
        {
            Debug.LogError("End Effector not assigned!");
            return;
        }

        if (!goalObject)
        {
            Debug.LogError("Goal Object not assigned!");
            return;
        }

        Vector3 startPosition = endEffector.transform.position; // Use end-effector position as start
        Vector3 goalPosition = goalObject.transform.position;

        // Step 1: Sample nodes within the grid
        sampledNodes = SampleNodes(sampleCount);

        // Step 2: Build PRM connections and find path
        List<Vector3> prmPath = FindPathPRM(startPosition, goalPosition);

        // Step 3: Visualize the path if successful
        if (prmPath != null)
        {
            path = prmPath;
            isPathGenerated = true; // Set the flag to true when path is found
            Debug.Log("Path found!");
        }
        else
        {
            Debug.Log("No path found.");
        }
    }

    void OnDrawGizmos()
    {
        if (!drawPath) return;

        if (sampledNodes != null)
        {
            Gizmos.color = Color.blue;
            foreach (var node in sampledNodes)
            {
                Gizmos.DrawSphere(node, 0.05f);
            }
        }

        if (path != null)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < path.Count - 1; i++)
            {
                Gizmos.DrawLine(path[i], path[i + 1]);
            }
        }
    }

    // Sample nodes within the grid, ensuring they are not in obstacles
    private List<Vector3> SampleNodes(int count)
    {
        List<Vector3> nodes = new List<Vector3>();
        int attempts = 0;

        while (nodes.Count < count && attempts < count * 2)
        {
            Vector3Int randomCell = new Vector3Int(
                Random.Range(0, gridManager.gridSize.x),
                Random.Range(0, gridManager.gridSize.y),
                Random.Range(0, gridManager.gridSize.z)
            );

            if (!gridManager.IsObstacle(randomCell.x, randomCell.y, randomCell.z))
            {
                Vector3 worldPos = gridManager.GridToWorld(randomCell.x, randomCell.y, randomCell.z);
                nodes.Add(worldPos);
            }

            attempts++;
        }

        return nodes;
    }

    private List<Vector3> FindPathPRM(Vector3 start, Vector3 goal)
    {
        sampledNodes.Add(start);
        sampledNodes.Add(goal);

        Dictionary<Vector3, List<Vector3>> graph = BuildGraph(sampledNodes, connectionRadius);

        return AStarPathfinding(graph, start, goal);
    }

    private Dictionary<Vector3, List<Vector3>> BuildGraph(List<Vector3> nodes, float radius)
    {
        Dictionary<Vector3, List<Vector3>> graph = new Dictionary<Vector3, List<Vector3>>();

        foreach (Vector3 node in nodes)
        {
            graph[node] = new List<Vector3>();

            foreach (Vector3 otherNode in nodes)
            {
                if (node != otherNode && Vector3.Distance(node, otherNode) <= radius && !IsPathBlocked(node, otherNode))
                {
                    graph[node].Add(otherNode);
                }
            }
        }

        return graph;
    }

    private bool IsPathBlocked(Vector3 start, Vector3 end)
    {
        // Step along the path between `start` and `end` in small increments
        Vector3 direction = (end - start).normalized;
        float distance = Vector3.Distance(start, end);
        int steps = Mathf.CeilToInt(distance / gridManager.cellSize);

        for (int i = 0; i <= steps; i++)
        {
            Vector3 samplePoint = start + direction * (i * gridManager.cellSize);
            Vector3Int cell = gridManager.WorldToGrid(samplePoint);

            if (gridManager.IsObstacle(cell.x, cell.y, cell.z))
            {
                return true;
            }
        }

        return false;
    }

    private List<Vector3> AStarPathfinding(Dictionary<Vector3, List<Vector3>> graph, Vector3 start, Vector3 goal)
    {
        HashSet<Vector3> openSet = new HashSet<Vector3> { start };
        Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
        Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float> { [start] = 0 };
        Dictionary<Vector3, float> fScore = new Dictionary<Vector3, float> { [start] = Heuristic(start, goal) };

        while (openSet.Count > 0)
        {
            Vector3 current = GetLowestScoreNode(openSet, fScore);

            if (current == goal)
            {
                return ReconstructPath(cameFrom, current);
            }

            openSet.Remove(current);

            foreach (Vector3 neighbor in graph[current])
            {
                float tentativeGScore = gScore[current] + Vector3.Distance(current, neighbor);

                if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + Heuristic(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        return null;
    }

    private Vector3 GetLowestScoreNode(HashSet<Vector3> openSet, Dictionary<Vector3, float> fScore)
    {
        Vector3 lowestNode = default;
        float lowestScore = Mathf.Infinity;

        foreach (Vector3 node in openSet)
        {
            if (fScore.TryGetValue(node, out float score) && score < lowestScore)
            {
                lowestScore = score;
                lowestNode = node;
            }
        }

        return lowestNode;
    }

    private List<Vector3> ReconstructPath(Dictionary<Vector3, Vector3> cameFrom, Vector3 current)
    {
        List<Vector3> path = new List<Vector3> { current };

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }

        return path;
    }

    private float Heuristic(Vector3 a, Vector3 b)
    {
        return Vector3.Distance(a, b);
    }
}
