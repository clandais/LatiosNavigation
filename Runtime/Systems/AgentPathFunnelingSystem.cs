using Latios;
using Latios.Transforms;
using LatiosNavigation.Authoring;
using LatiosNavigation.Utils;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace LatiosNavigation.Systems
{
    public partial struct AgentPathFunnelingSystem : ISystem
    {
        EntityQuery          m_query;
        LatiosWorldUnmanaged m_latiosWorld;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_latiosWorld = state.GetLatiosWorldUnmanaged();
            m_query = state.Fluent().WithAspect<TransformAspect>().With<NavmeshAgentTag>().With<AgentDestination>()
                .With<AgentPath>().With<AgentPathEdge>().With<AgentPathPoint>().WithEnabled<AgentHasEdgePathTag>()
                .Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = m_latiosWorld.syncPoint.CreateEntityCommandBuffer();
            state.Dependency = new FunnelJob
            {
                Ecb = ecb.AsParallelWriter()
            }.ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        partial struct FunnelJob : IJobEntity
        {
            public EntityCommandBuffer.ParallelWriter Ecb;

            void Execute(Entity entity, [EntityIndexInQuery] int entityIndex, TransformAspect transformAspect,
                ref AgentPath agentPath, in AgentDestination destination, in DynamicBuffer<AgentPathEdge> portals,
                ref DynamicBuffer<AgentPathPoint> pathPoints)
            {
                pathPoints.Clear();
                var start = transformAspect.worldPosition;
                var end = destination.Position;
                if (portals.IsEmpty)
                {
                    pathPoints.Add(new AgentPathPoint
                    {
                        Position = start
                    });

                    pathPoints.Add(new AgentPathPoint
                    {
                        Position = end
                    });

                    agentPath.PathLength = 2;
                    agentPath.PathIndex  = 0;
                    Ecb.SetComponentEnabled<AgentHasEdgePathTag>(entityIndex, entity, false);
                    Debug.Log($"No portals for agent {entity.Index}, direct path from {start} to {end}.");
                    return;
                }

                pathPoints.Add(new AgentPathPoint
                {
                    Position = start
                });

                var portalCount = portals.Length;
                var portalApex =
                    //start;
                    portals[0].PortalVertex1; // Start with the first portal's left vertex as the apex

                var portalLeft = portals[0].PortalVertex1;
                var portalRight = portals[0].PortalVertex2;
                int leftIndex = 0, rightIndex = 0;
                var i = 1;
                while (i < portalCount)
                {
                    var left = portals[i].PortalVertex1;
                    var right = portals[i].PortalVertex2;

                    // Right leg
                    if (TriMath.TriArea2(portalApex, portalRight, right) <= 0f)
                    {
                        if (Vequals(portalApex, portalRight) || TriMath.TriArea2(portalApex, portalLeft, right) > 0f)
                        {
                            // Tighten the funnel
                            portalRight = right;
                            rightIndex  = i;
                        }
                        else
                        {
                            // Right over left
                            pathPoints.Add(new AgentPathPoint
                            {
                                Position = portalLeft
                            });

                            // path.Add(portalLeft);
                            portalApex = portalLeft;
                            i          = leftIndex + 1;

                            // Reset
                            portalLeft  = portalApex;
                            portalRight = portalApex;
                            leftIndex   = i - 1;
                            rightIndex  = i - 1;
                            continue;
                        }
                    }

                    // Left leg
                    if (TriMath.TriArea2(portalApex, portalLeft, left) >= 0f)
                    {
                        if (Vequals(portalApex, portalLeft) || TriMath.TriArea2(portalApex, portalRight, left) < 0f)
                        {
                            // Tighten the funnel
                            portalLeft = left;
                            leftIndex  = i;
                        }
                        else
                        {
                            // Left over right
                            pathPoints.Add(new AgentPathPoint
                            {
                                Position = portalRight
                            });

                            //path.Add(portalRight);
                            portalApex = portalRight;
                            i          = rightIndex + 1;

                            // Reset
                            portalLeft  = portalApex;
                            portalRight = portalApex;
                            leftIndex   = i - 1;
                            rightIndex  = i - 1;
                            continue;
                        }
                    }

                    i++;
                }


                pathPoints.Add(new AgentPathPoint
                {
                    Position = end
                });

                agentPath.PathLength = pathPoints.Length;
                agentPath.PathIndex  = 0;
                Ecb.SetComponentEnabled<AgentHasEdgePathTag>(entityIndex, entity, false);
            }
        }

        static bool Vequals(float3 a, float3 b) => math.distancesq(a.xz, b.xz) < .001f * .001f;
    }
}