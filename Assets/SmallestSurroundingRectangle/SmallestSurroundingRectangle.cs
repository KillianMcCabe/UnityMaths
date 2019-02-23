using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmallestSurroundingRectangle : MonoBehaviour
{
    public float Range = 20f;
    public int NumberOfTestPoints = 4;

    struct Edge
    {
        public Vector3 A;
        public Vector3 B;

        public Edge(Vector3 a, Vector3 b)
        {
            A = a;
            B = b;
        }
    }

    [SerializeField]
    LineRenderer _convexHullLineRenderer = null;

    [SerializeField]
    GameObject _boundingBox = null;

    [SerializeField]
    Transform _testPointsTransform = null;

    List<Vector3> _points;

    // Start is called before the first frame update
    void Start()
    {
        RunTest();
    }

    public void RunTest()
    {
        // Cleanup
        for (int i = _testPointsTransform.childCount - 1; i >= 0; i--)
        {
            GameObject.Destroy(_testPointsTransform.GetChild(i).gameObject);
        }

        // spawn test points
        _points = new List<Vector3>();
        for (int i = 0; i < NumberOfTestPoints; i++)
        {
            // generate random point
            Vector3 p = new Vector3(UnityEngine.Random.Range(-Range, Range), 0, UnityEngine.Random.Range(-Range, Range));
            _points.Add(p);

            // Create sphere to render point
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.SetParent(_testPointsTransform);
            sphere.transform.position = p;
        }

        CalculateBoundingBox();
    }

    private void CalculateBoundingBox()
    {
        // Compute the convex hull of the cloud
        List<Vector3> convexHull = ConvexHull(_points);
        DrawLine(convexHull);

        List<Edge> convexHullNormals = ComputeEdges(convexHull);

        float minBoundsAngle = 0;
        Edge bestEdge = new Edge();
        Bounds minBounds = new Bounds();
        minBounds.Expand(Mathf.Infinity);

        // For each edge of the convex hull:
        foreach (Edge edge in convexHullNormals)
        {
            // Compute the edge orientation (with arctan?)
            Vector3 ab = (edge.A - edge.B).normalized;
            float angle = Mathf.Atan2(ab.z, ab.x) * Mathf.Rad2Deg;

            // Rotate the convex hull using this orientation
            List<Vector3> rotatedConvexHull = RotatePoints(convexHull, angle);

            // Compute the bounding rectangle area of the rotated convex hull
            Bounds b = new Bounds();
            foreach (Vector3 p in rotatedConvexHull)
            {
                b.Encapsulate(p);
            }

            // Store the bounds and orientation corresponding to the minimum area found
            if (b.size.x * b.size.z < minBounds.size.x * minBounds.size.z)
            {
                minBounds = b;
                bestEdge = edge;
                minBoundsAngle = angle;
            }
        }

        Debug.DrawLine(bestEdge.A, bestEdge.B, Color.cyan, 10, false);
        DrawBounds(minBounds, -minBoundsAngle);
    }

    private List<Vector3> ConvexHull(List<Vector3> points)
    {
        List<Vector3> convexHullPoints = new List<Vector3>();

        // get point with max X value
        float maxXValue = -Mathf.Infinity;
        int maxXIndex = -1;
        for (int i = 0; i < points.Count; i++)
        {
            if (points[i].x > maxXValue)
            {
                maxXIndex = i;
                maxXValue = points[i].x;
            }
        }

        // Create sphere to render point
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.SetParent(_testPointsTransform);
        sphere.transform.position = points[maxXIndex];
        sphere.transform.localScale = new Vector3(1.5f, 1.5f, 1.5f);

        // start calculating the hull from the point with the max X value
        convexHullPoints.Add(points[maxXIndex]);
        Vector3 startPosition = points[maxXIndex];
        // find next point with smallest angle
        while (true)
        {
            Vector3 lastPoint = convexHullPoints[convexHullPoints.Count-1];
            int smallestArcIndex = GetIndexOfPointWithSmallestArcForConvexHull(points, lastPoint);
            Vector3 newPoint = points[smallestArcIndex];
            // stop calculating when we get back to starting point
            if (startPosition == newPoint || smallestArcIndex == -1)
            {
                break;
            }

            convexHullPoints.Add(newPoint);
            points.RemoveAt(smallestArcIndex);
        }

        return convexHullPoints;
    }

    private int GetIndexOfPointWithSmallestArcForConvexHull(List<Vector3> points, Vector3 p)
    {
        int bestIndex = -1;
        float bestAngle = Mathf.Infinity;
        Debug.DrawLine(p, p+p.normalized, Color.green, 1, false);
        for (int i = 0; i < points.Count; i++)
        {
            Debug.DrawLine(p, p+(points[i] - p), Color.red, 1, false);
            float angle = Vector3.SignedAngle(points[i] - p, -p, Vector3.up) + 180f;
            if (angle < bestAngle && !Mathf.Approximately(angle, 180))
            {
                bestAngle = angle;
                bestIndex = i;
            }
        }

        if (bestIndex > -1)
        {
            Debug.DrawLine(p, p+(points[bestIndex] - p), Color.green, 1, false);
        }

        return bestIndex;
    }

    private List<Edge> ComputeEdges(List<Vector3> points)
    {
        List<Edge> edges = new List<Edge>();

        for (int i = 0; i < points.Count - 1; i++)
        {
            edges.Add(new Edge(points[i], points[i+1]));
        }

        // add normal between first and last point
        edges.Add(new Edge(points[points.Count-1], points[0]));

        return edges;
    }

    private List<Vector3> RotatePoints(List<Vector3> points, float angle)
    {
        List<Vector3> rotatedPoints = new List<Vector3>();
        for (int i = 0; i < points.Count; i++)
        {
            rotatedPoints.Add(Quaternion.Euler(0, angle, 0) * points[i]);
        }
        return rotatedPoints;
    }

    private void DrawLine(List<Vector3> points)
    {
        List<Vector3> linePoints = points;
        linePoints.Add(points[0]);
        _convexHullLineRenderer.positionCount = linePoints.Count;
        _convexHullLineRenderer.SetPositions(linePoints.ToArray());
    }

    private void DrawBounds(Bounds bounds, float angle)
    {
        _boundingBox.transform.position = bounds.center;
        _boundingBox.transform.RotateAround(Vector3.zero, Vector3.up, angle);
        _boundingBox.transform.localScale = new Vector3(bounds.size.x, Mathf.Max(bounds.size.y, 1), bounds.size.z);
    }

    private void OnGUI()
    {
        if (GUILayout.Button("Run Test"))
        {
            RunTest();
        }
    }
}
