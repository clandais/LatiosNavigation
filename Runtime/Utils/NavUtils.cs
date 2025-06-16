using System;
using LatiosNavigation.Authoring;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace LatiosNavigation.Utils
{
    public static class NavUtils
    {
        public static void Debug(ref NavMeshSurfaceBlob navMeshSurfaceBlob)
        {
            for (var i = 0; i < navMeshSurfaceBlob.Triangles.Length; i++)
            {
                var triangle = navMeshSurfaceBlob.Triangles[i];
                UnityEngine.Debug.DrawLine(triangle.pointA, triangle.pointB, Color.green);
                UnityEngine.Debug.DrawLine(triangle.pointB, triangle.pointC, Color.green);
                UnityEngine.Debug.DrawLine(triangle.pointC, triangle.pointA, Color.green);
            }
        }

        public static bool FindTriangleContainingPoint(float3 position, ref NavMeshSurfaceBlob navMeshSurfaceBlob,
            out int containingTriangleIndex)
        {
            containingTriangleIndex = -1;
            for (var i = 0; i < navMeshSurfaceBlob.Triangles.Length; i++)
            {
                var triangle = navMeshSurfaceBlob.Triangles[i];
                if (TriMath.IsPointInTriangle(position, triangle))
                {
                    containingTriangleIndex = i;
                    return true;
                }
            }

            return false;
        }

        public static NavTriangle GetTriangleByIndex(int index, ref NavMeshSurfaceBlob navMeshSurfaceBlob)
        {
            if (index < 0 || index >= navMeshSurfaceBlob.Triangles.Length)
                throw new IndexOutOfRangeException("Triangle index is out of range.");

            return navMeshSurfaceBlob.Triangles[index];
        }

        public static bool TryGetSharedPortalVertices(in NavTriangle t1, in NavTriangle t2, out float3 p1,
            out float3 p2)
        {
            p1 = float3.zero;
            p2 = float3.zero;
            var foundCount = 0;
            var t1Indices = new NativeArray<int>(3, Allocator.Temp);
            t1Indices[0] = t1.iA;
            t1Indices[1] = t1.iB;
            t1Indices[2] = t1.iC;
            var t1Coords = new NativeArray<float3>(3, Allocator.Temp);
            t1Coords[0] = t1.pointA;
            t1Coords[1] = t1.pointB;
            t1Coords[2] = t1.pointC;
            var t2Indices = new NativeArray<int>(3, Allocator.Temp);
            t2Indices[0] = t2.iA;
            t2Indices[1] = t2.iB;
            t2Indices[2] = t2.iC;
            var t2Coords = new NativeArray<float3>(3, Allocator.Temp);
            t2Coords[0] = t2.pointA;
            t2Coords[1] = t2.pointB;
            t2Coords[2] = t2.pointC;
            for (var i = 0; i < 3; i++)
            {
                var isShared = false;
                for (var j = 0; j < 3; j++)
                    if (t1Indices[i] == t2Indices[j])
                    {
                        isShared = true;
                        break;
                    }

                if (isShared)
                {
                    if (foundCount == 0)
                        p1                       = t1Coords[i];
                    else if (foundCount == 1) p2 = t1Coords[i];

                    foundCount++;
                    if (foundCount == 2) break;
                }
            }

            var currentCenter = t1.Centroid;
            if (TriMath.TriArea2(currentCenter, p1, p2) <= 0f) (p1, p2) = (p2, p1);
            t1Indices.Dispose();
            t1Coords.Dispose();
            t2Indices.Dispose();
            t2Coords.Dispose();
            return foundCount == 2;
        }
    }
}