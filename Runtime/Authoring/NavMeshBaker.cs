using System;
using System.Collections.Generic;
using Latios.Authoring;
using Unity.AI.Navigation;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;

namespace LatiosNavigation.Authoring
{
    [DisableAutoCreation]
    public class NavMeshBaker : SmartBaker<NavMeshSurface, NavMeshSurfaceBakeItem> { }

    [TemporaryBakingType]
    public struct NavMeshSurfaceBakeItem : ISmartBakeItem<NavMeshSurface>
    {
        SmartBlobberHandle<NavMeshSurfaceBlob> m_navMeshSurfaceHandle;

        public bool Bake(NavMeshSurface authoring, IBaker baker)
        {
            var entity = baker.GetEntity(authoring, TransformUsageFlags.Dynamic);
            baker.AddComponent<NavMeshSurfaceBlobReference>(entity);
            m_navMeshSurfaceHandle = baker.RequestCreateBlobAsset(authoring);
            return true;
        }

        public void PostProcessBlobRequests(EntityManager entityManager, Entity entity)
        {
            var blobAsset = m_navMeshSurfaceHandle.Resolve(entityManager);

            entityManager.SetComponentData(entity, new NavMeshSurfaceBlobReference
            {
                NavMeshSurfaceBlob = blobAsset
            });
        }
    }

    public static class NavMeshSmartBlobberBakerExtensions
    {
        public static SmartBlobberHandle<NavMeshSurfaceBlob>
            RequestCreateBlobAsset(this IBaker baker, NavMeshSurface authoring) =>
            baker.RequestCreateBlobAsset<NavMeshSurfaceBlob, NaveMeshSmartBlobberRequestFilter>(
                new NaveMeshSmartBlobberRequestFilter
                {
                    surface = authoring
                });
    }


    public struct NavMeshSurfaceBlobReference : IComponentData
    {
        public BlobAssetReference<NavMeshSurfaceBlob> NavMeshSurfaceBlob;
    }

    public struct NavMeshSurfaceBlob
    {
        public BlobArray<NavTriangle> Triangles;
        public BlobArray<int>         AdjacencyIndices;
        public BlobArray<int2>        AdjacencyOffsets; // x = start index, y = count
    }

    public struct NavTriangle
    {
        public int    iA;
        public int    iB;
        public int    iC;
        public float3 pointA;
        public float3 pointB;
        public float3 pointC;

        public float3 Centroid => (pointA + pointB + pointC) / 3f;
        public float Radius => GetBoundingRadius(pointA, pointB, pointC);

        float GetBoundingRadius(float3 pointA, float3 pointB, float3 pointC)
        {
            var boundingRadius = 0f;
            var center = Centroid;
            var d = math.distance(center, pointA);
            if (d > boundingRadius) boundingRadius = d;
            d = math.distance(center, pointB);
            if (d > boundingRadius) boundingRadius = d;
            d = math.distance(center, pointC);
            if (d > boundingRadius) boundingRadius = d;
            return boundingRadius;
        }
    }

    // public struct NavMeshSurfaceComponent : IComponentData { }


    [TemporaryBakingType]
    public struct TriangleElementInput : IBufferElementData
    {
        public int    iA;
        public int    iB;
        public int    iC;
        public float3 pointA;
        public float3 pointB;
        public float3 pointC;
    }

    // [TemporaryBakingType]
    // public struct NavMeshSmartBlobberResult : IComponentData
    // {
    //     public UnsafeUntypedBlobAssetReference Blob;
    // }

    public struct NaveMeshSmartBlobberRequestFilter : ISmartBlobberRequestFilter<NavMeshSurfaceBlob>
    {
        public NavMeshSurface surface;


        public bool Filter(IBaker baker, Entity blobBakingEntity)
        {
            if (surface == null)
            {
                Debug.LogError(
                    $" NavMeshSurfaceBakeItem: NavMeshSurface is null for entity {blobBakingEntity}. Ensure the NavMeshSurface component is assigned.");

                return false;
            }

            // baker.DependsOn(surface);

            var triangulation = NavMesh.CalculateTriangulation();

            if (triangulation.vertices.Length == 0 || triangulation.indices.Length == 0)
            {
                Debug.LogError(
                    $" NavMeshSurfaceBakeItem: No triangles found for entity {blobBakingEntity}. Ensure the NavMeshSurface has been baked.");

                return false;
            }


            var uniqueVerts = new Dictionary<float3, int>();
            var remappedVerts = new List<float3>();
            var remappedTriangles = new List<Triangle>();

            foreach (var v in triangulation.vertices)
                if (!uniqueVerts.TryGetValue(v, out var index))
                {
                    index          = remappedVerts.Count;
                    uniqueVerts[v] = index;
                    remappedVerts.Add(v);
                }

            for (var i = 0; i < triangulation.indices.Length; i += 3)
            {
                var a = uniqueVerts[triangulation.vertices[triangulation.indices[i]]];
                var b = uniqueVerts[triangulation.vertices[triangulation.indices[i + 1]]];
                var c = uniqueVerts[triangulation.vertices[triangulation.indices[i + 2]]];

                remappedTriangles.Add(new Triangle
                {
                    VertexA = a,
                    VertexB = b,
                    VertexC = c
                });
            }

            var buffer = baker.AddBuffer<TriangleElementInput>(blobBakingEntity);


            for (var i = 0; i < remappedTriangles.Count; i++)
                buffer.Add(new TriangleElementInput
                {
                    iA     = remappedTriangles[i].VertexA,
                    iB     = remappedTriangles[i].VertexB,
                    iC     = remappedTriangles[i].VertexC,
                    pointA = remappedVerts[remappedTriangles[i].VertexA],
                    pointB = remappedVerts[remappedTriangles[i].VertexB],
                    pointC = remappedVerts[remappedTriangles[i].VertexC]
                });

            return true;
        }
    }

    public struct Triangle
    {
        public int VertexA;
        public int VertexB;
        public int VertexC;
    }

    public struct Edge : IEquatable<Edge>
    {
        public int VertexA;
        public int VertexB;

        public bool Equals(Edge other) => (VertexA == other.VertexA && VertexB == other.VertexB) ||
                                          (VertexA == other.VertexB && VertexB == other.VertexA);

        public override bool Equals(object obj) => obj is Edge other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                return (VertexA * 397) ^ VertexB;
            }
        }
    }
}