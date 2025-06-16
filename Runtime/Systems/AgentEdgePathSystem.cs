using System.Collections.Generic;
using Latios;
using Latios.Transforms;
using LatiosNavigation.Authoring;
using LatiosNavigation.Utils;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace LatiosNavigation.Systems
{
    [RequireMatchingQueriesForUpdate]
    public partial struct AgentEdgePathSystem : ISystem
    {
        EntityQuery          m_query;
        LatiosWorldUnmanaged m_latiosWorld;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_latiosWorld = state.GetLatiosWorldUnmanaged();
            m_query = state.Fluent()
                .WithAspect<TransformAspect>()
                .With<NavmeshAgentTag>()
                .With<NavMeshAgent>()
                .With<AgentDestination>()
                .With<AgentPath>()
                .With<AgentPathEdge>()
                .WithEnabled<AgenPathRequestedTag>()
                .Build();


            state.RequireForUpdate<NavMeshSurfaceBlobReference>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var navMeshSurfaceBlob =
                m_latiosWorld.sceneBlackboardEntity.GetComponentData<NavMeshSurfaceBlobReference>();

            var ecb = m_latiosWorld.syncPoint.CreateEntityCommandBuffer();

            state.Dependency = new PathJob
            {
                Ecb                = ecb.AsParallelWriter(),
                NavMeshSurfaceBlob = navMeshSurfaceBlob
            }.ScheduleParallel(m_query, state.Dependency);
        }


        [BurstCompile]
        partial struct PathJob : IJobEntity
        {
            [BurstCompile]
            struct QueueElement
            {
                public int Index;
                public int Cost;
            }

            [BurstCompile]
            struct CostComparer : IComparer<QueueElement>
            {
                public int Compare(QueueElement x,
                    QueueElement y) => x.Cost.CompareTo(y.Cost);
            }

            [ReadOnly] public NavMeshSurfaceBlobReference        NavMeshSurfaceBlob;
            public            EntityCommandBuffer.ParallelWriter Ecb;

            void Execute(Entity entity, [EntityIndexInQuery] int idx,
                TransformAspect transform,
                in NavMeshAgent navmeshAgent,
                in AgentDestination destination,
                ref AgentPath agentPath,
                ref DynamicBuffer<AgentPathEdge> buffer)
            {
                ref var blobAsset = ref NavMeshSurfaceBlob.NavMeshSurfaceBlob.Value;
                var destinationPosition = destination.Position; // Target position for the funnel algorithm

                // Determine start and goal triangles for A*
                if (!NavUtils.FindTriangleContainingPoint(transform.worldPosition, ref blobAsset,
                        out var startTriangleIndex))
                {
                    Debug.LogWarning($"Pathfinding: Agent at {transform.worldPosition} is not on the navmesh.");
                    buffer.Clear();
                    return;
                }

                if (!NavUtils.FindTriangleContainingPoint(destinationPosition, ref blobAsset,
                        out var goalTriangleIndex))
                {
                    Debug.LogWarning($"Pathfinding: Destination at {destinationPosition} is not on the navmesh.");
                    buffer.Clear();
                    return;
                }

                if (startTriangleIndex == goalTriangleIndex)
                {
                    // Agent and destination are in the same triangle.
                    // Path is just start to end point. Funnel algorithm might still be used with agent pos and target pos.
                    // For now, clear buffer, or add a single segment if your funnel expects it.
                    buffer.Clear();
                    // Add a "degenerate" portal or handle in funnel algorithm
                    buffer.Add(new AgentPathEdge
                    {
                        PortalVertex1 = transform.worldPosition, PortalVertex2 = destinationPosition
                    });

                    return;
                }


                var priorityQueue = new NativeMinHeap<QueueElement, CostComparer>(
                    blobAsset.Triangles.Length, Allocator.Temp);

                priorityQueue.Enqueue(new QueueElement
                {
                    Index = startTriangleIndex, // Start A* from the agent's current triangle
                    Cost  = 0
                });


                var cameFrom = new NativeParallelHashMap<int, int>(blobAsset.Triangles.Length, Allocator.Temp);
                // cameFrom.TryAdd(startTriangleIndex, startTriangleIndex); // No predecessor for start
                var costSoFar = new NativeParallelHashMap<int, int>(blobAsset.Triangles.Length, Allocator.Temp);
                costSoFar.TryAdd(startTriangleIndex, 0);

                var foundGoal = false;
                while (priorityQueue.TryDequeue(out var element))
                {
                    if (element.Index == goalTriangleIndex)
                    {
                        foundGoal = true;
                        break; // Found the goal triangle
                    }

                    var offsetData = blobAsset.AdjacencyOffsets[element.Index];
                    var currentTriangleForCost = NavUtils.GetTriangleByIndex(element.Index, ref blobAsset);

                    for (var i = offsetData.x; i < offsetData.x + offsetData.y; i++)
                    {
                        var neighborIndex = blobAsset.AdjacencyIndices[i];
                        var neighborTriangleForCost = NavUtils.GetTriangleByIndex(neighborIndex, ref blobAsset);
                        // Using distance between centers for A* heuristic/cost
                        var newCost = costSoFar[element.Index] + (int)math.distance(
                            currentTriangleForCost.Centroid, neighborTriangleForCost.Centroid) * 10;


                        if (!costSoFar.TryGetValue(neighborIndex, out var existingCost) || newCost < existingCost)
                        {
                            costSoFar[neighborIndex] = newCost;
                            priorityQueue.Enqueue(new QueueElement
                            {
                                Index = neighborIndex,
                                // Heuristic could be added here for A* (e.g., distance to goalTriangleIndex center)
                                Cost = newCost
                            });

                            cameFrom[neighborIndex] = element.Index;
                        }
                    }
                }


                buffer.Clear(); // Clear previous path

                if (!foundGoal)
                {
                    Debug.LogWarning("Pathfinding failed to find a path to the goal triangle.");
                    cameFrom.Dispose();
                    costSoFar.Dispose();
                    priorityQueue.Dispose();
                    return;
                }

                // Reconstruct the path of portals
                var tempPathPortals =
                    new NativeList<AgentPathEdge>(Allocator.Temp);

                var currentPathIndex = goalTriangleIndex;

                while (currentPathIndex != startTriangleIndex)
                {
                    if (!cameFrom.TryGetValue(currentPathIndex, out var previousPathIndex))
                    {
                        Debug.LogError("Path reconstruction failed: cameFrom map incomplete.");
                        tempPathPortals.Dispose();
                        cameFrom.Dispose();
                        costSoFar.Dispose();
                        priorityQueue.Dispose();
                        return;
                    }

                    var triCurrent = NavUtils.GetTriangleByIndex(currentPathIndex, ref blobAsset);
                    var triPrevious = NavUtils.GetTriangleByIndex(previousPathIndex, ref blobAsset);

                    if (NavUtils.TryGetSharedPortalVertices(triPrevious, triCurrent, out var p1, out var p2))
                    {
                        // Add portal between triPrevious and triCurrent
                        tempPathPortals.Add(new AgentPathEdge
                        {
                            PortalVertex1 =
                                p1, // Order might matter for funnel, ensure consistency or let funnel handle it
                            PortalVertex2 = p2
                        });
                    }
                    else
                    {
                        Debug.LogError(
                            $"Failed to find shared portal between triangle {currentPathIndex} and {previousPathIndex}.");

                        tempPathPortals.Dispose();
                        cameFrom.Dispose();
                        costSoFar.Dispose();
                        priorityQueue.Dispose();
                        return;
                    }

                    currentPathIndex = previousPathIndex;
                }


                buffer.Add(new AgentPathEdge
                {
                    PortalVertex1 = transform.worldPosition,
                    PortalVertex2 = transform.worldPosition
                }); // Add start position as first portal vertex

                // Add portals to the buffer in the correct order (start to goal)
                for (var i = tempPathPortals.Length - 1; i >= 0; i--)
                {
                    var portal = tempPathPortals[i];
                    var leftVertex = portal.PortalVertex1;
                    var rightVertex = portal.PortalVertex2;
                    //
                    var direction = math.normalize(rightVertex - leftVertex) * navmeshAgent.Radius;
                    leftVertex  += direction; // Move left vertex slightly towards right vertex
                    rightVertex -= direction; // Move right vertex slightly towards left vertex

                    portal.PortalVertex1 = leftVertex;
                    portal.PortalVertex2 = rightVertex;
                    buffer.Add(portal);
                }

                buffer.Add(new AgentPathEdge
                {
                    PortalVertex1 = destinationPosition,
                    PortalVertex2 = destinationPosition
                }); // Add destination position as last portal vertex

                tempPathPortals.Dispose();
                cameFrom.Dispose();
                costSoFar.Dispose();
                priorityQueue.Dispose();

                Ecb.SetComponentEnabled<AgenPathRequestedTag>(idx, entity, false);
                Ecb.SetComponentEnabled<AgentHasEdgePathTag>(idx, entity, true);

                if (buffer.Length == 0 && startTriangleIndex != goalTriangleIndex)
                    Debug.LogWarning("Pathfinding resulted in an empty portal list for a non-trivial path.");
            }
        }
    }
}