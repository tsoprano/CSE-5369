using System.Collections.Generic;
using UnityEngine;

public class PRMPathfinderProj : MonoBehaviour
{
    public int sampleCount = 50; // Number of nodes to sample for PRM
    public float connectionRadius = 0.5f; // Radius within which nodes can be connected
    public GridManagerProj gridManager; // Reference to GridManager
    public GameObject endEffector; // End effector GameObject (robot's current position)
    private List<Vector3> sampledNodes;
    private Dictionary<Vector3, List<Vector3>> graph; // PRM graph
    public List<Vector3> path;
    public bool isPathGenerated = false; // Flag to indicate path generation

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

        // Step 1: Sample nodes within the grid
        sampledNodes = SampleNodes(sampleCount);

        // Step 2: Build PRM graph
        graph = BuildGraph(sampledNodes, connectionRadius);
    }

    public void GeneratePath(Vector3 startPosition, Vector3 goalPosition)
    {
        if (graph == null)
        {
            Debug.LogError("PRM graph not initialized!");
            return;
        }
        // startPosition = new Vector3(0.1f, 0.0f, 0.25f);
        goalPosition = new Vector3(0.018f, -0.055f, 0.374f);

        // Add start and goal nodes dynamically
        sampledNodes.Add(startPosition);
        sampledNodes.Add(goalPosition);

        Debug.Log($"=== startPosition: {startPosition}");
        graph[startPosition] = ConnectNodeToGraph(startPosition);

        Debug.Log($"=== goalPosition: {goalPosition}");
        graph[goalPosition] = ConnectNodeToGraph(goalPosition);

        // Step 3: Find a path using A* or similar
        path = AStarPathfinding(graph, startPosition, goalPosition);
        if (path == null)
        {
            Debug.Log("path is null");
        }
        // foreach (Vector3 neighbor in path)
        // {
        //     Debug.Log(neighbor); 
        // }

        // Remove dynamic nodes after pathfinding
        sampledNodes.Remove(startPosition);
        sampledNodes.Remove(goalPosition);
        graph.Remove(startPosition);
        graph.Remove(goalPosition);

        isPathGenerated = path != null;
    }

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

    private Dictionary<Vector3, List<Vector3>> BuildGraph(List<Vector3> nodes, float radius)
    {
        Dictionary<Vector3, List<Vector3>> graph = new Dictionary<Vector3, List<Vector3>>();

        foreach (Vector3 node in nodes)
        {
            graph[node] = ConnectNodeToGraph(node);
        }

        return graph;
    }

    private List<Vector3> ConnectNodeToGraph(Vector3 node)
    {
        List<Vector3> neighbors = new List<Vector3>();

        foreach (Vector3 otherNode in sampledNodes)
        {
            if (node != otherNode && Vector3.Distance(node, otherNode) <= connectionRadius && !IsPathBlocked(node, otherNode))
            {
                neighbors.Add(otherNode);
            }
        }

        return neighbors;
    }

    private bool IsPathBlocked(Vector3 start, Vector3 end)
    {
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

    // private List<Vector3> AStarPathfinding(Dictionary<Vector3, List<Vector3>> graph, Vector3 start, Vector3 goal)
    // {
    //     HashSet<Vector3> openSet = new HashSet<Vector3> { start };
    //     Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
    //     Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float> { [start] = 0 };
    //     Dictionary<Vector3, float> fScore = new Dictionary<Vector3, float> { [start] = Heuristic(start, goal) };

    //     while (openSet.Count > 0)
    //     {
    //         Vector3 current = GetLowestScoreNode(openSet, fScore);
    //         Debug.Log($"=== current: {current}");

    //         if (current == goal)
    //         {
    //             return ReconstructPath(cameFrom, current);
    //         }

    //         openSet.Remove(current);

    //         foreach (Vector3 neighbor in graph[current])
    //         {
    //             float tentativeGScore = gScore[current] + Vector3.Distance(current, neighbor);

    //             if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
    //             {
    //                 cameFrom[neighbor] = current;
    //                 gScore[neighbor] = tentativeGScore;
    //                 fScore[neighbor] = tentativeGScore + Heuristic(neighbor, goal);

    //                 if (!openSet.Contains(neighbor))
    //                 {
    //                     openSet.Add(neighbor);
    //                 }
    //             }
    //         }
    //     }

    //     return null;
    // }

    private List<Vector3> AStarPathfinding(Dictionary<Vector3, List<Vector3>> graph, Vector3 start, Vector3 goal)
{
    if (!graph.ContainsKey(start))
    {
        Debug.LogError($"Start node {start} is not in the graph.");
        return null;
    }

    if (!graph.ContainsKey(goal))
    {
        Debug.LogError($"Goal node {goal} is not in the graph.");
        return null;
    }

    HashSet<Vector3> openSet = new HashSet<Vector3> { start };
    Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();
    Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float> { [start] = 0 };
    Dictionary<Vector3, float> fScore = new Dictionary<Vector3, float> { [start] = Heuristic(start, goal) };

    while (openSet.Count > 0)
    {
        Vector3 current = GetLowestScoreNode(openSet, fScore);
        Debug.Log($"Processing Node: {current}");

        // Use threshold-based comparison for the goal
        if (Vector3.Distance(current, goal) < 0.5f) // Replace 0.01f with desired precision
        {
            Debug.Log("Goal reached!");
            return ReconstructPath(cameFrom, current);
        }

        openSet.Remove(current);

        foreach (Vector3 neighbor in graph[current])
        {
            // Debug.Log($"Checking neighbor: {neighbor}");
            float tentativeGScore = gScore[current] + Vector3.Distance(current, neighbor);

            if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
            {
                // Debug.Log($"Updating neighbor: {neighbor}");
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

    Debug.LogError("Path not found.");
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
        Debug.Log("===ReconstructPath===>");
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
