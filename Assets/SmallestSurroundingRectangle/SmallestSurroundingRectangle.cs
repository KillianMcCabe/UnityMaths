using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmallestSurroundingRectangle : MonoBehaviour
{
    const float Range = 20f;
    const int NumberOfTestPoints = 10;

    List<Vector3> _points;

    public LineRenderer _convexHullLineRenderer;

    // Start is called before the first frame update
    void Start()
    {
        SetupTest();
        CalculateBoundingBox(); 
    }

    private void SetupTest()
    {
        // spawn test points
        _points = new List<Vector3>();
        for (int i = 0; i < NumberOfTestPoints; i++)
        {
            // generate random point
            Vector3 p = new Vector3(Random.Range(-Range, Range), 0, Random.Range(-Range, Range));
            _points.Add(p);

            // Create sphere to render point
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = p;
        }
    }

    private void CalculateBoundingBox()
    {
        // Compute the convex hull of the cloud
        List<Vector3> convexHull = ConvexHull(_points);
        DrawLine(convexHull);

        // For each edge of the convex hull:
        // compute the edge orientation (with arctan),
        // rotate the convex hull using this orientation in order to compute easily the bounding rectangle area with min/max of x/y of the rotated convex hull,
        // Store the orientation corresponding to the minimum area found,
        // Return the rectangle corresponding to the minimum area found.
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
            Debug.Log("bestAngle: " + points[bestIndex] + "(" + bestAngle + ")");
        }

        return bestIndex;
    }

    private int GetIndexOfPointWithSmallestArcForOutline(List<Vector3> points, Vector3 focusedPoint)
    {
        int bestIndex = -1;
        float bestAngle = Mathf.Infinity;

        Debug.DrawLine(focusedPoint, focusedPoint + focusedPoint.normalized, Color.green, 1, false);
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 towardsPoint = focusedPoint + (points[i] - focusedPoint);
            Debug.DrawLine(focusedPoint.normalized, towardsPoint, Color.red, 1, false);
            float angle = Vector3.SignedAngle(towardsPoint.normalized, -focusedPoint.normalized, Vector3.up) + 180f;
            if (angle < bestAngle && !Mathf.Approximately(angle, 180))
            {
                bestAngle = angle;
                bestIndex = i;
            }
        }

        if (bestIndex > -1)
        {
            Debug.DrawLine(focusedPoint, focusedPoint+(points[bestIndex] - focusedPoint), Color.green, 1, false);
            Debug.Log("bestAngle: " + points[bestIndex] + "(" + bestAngle + ")");
        }

        return bestIndex;
    }

    private void DrawLine(List<Vector3> points)
    {
        List<Vector3> linePoints = points;
        linePoints.Add(points[0]);
        _convexHullLineRenderer.positionCount = linePoints.Count;
        _convexHullLineRenderer.SetPositions(linePoints.ToArray());
    }
}
