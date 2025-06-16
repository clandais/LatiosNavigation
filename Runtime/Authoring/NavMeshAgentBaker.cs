using Unity.Entities;
using Unity.Mathematics;

namespace LatiosNavigation.Authoring
{
    [DisableAutoCreation]
    public class NavMeshAgentBaker : Baker<UnityEngine.AI.NavMeshAgent>
    {
        public override void Bake(UnityEngine.AI.NavMeshAgent authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<NavmeshAgentTag>(entity);

            AddComponent(entity, new NavMeshAgent
            {
                Radius = authoring.radius
            });

            AddComponent(entity, new AgentDestination
            {
                Position = authoring.transform.position
            });

            AddBuffer<AgentPathEdge>(entity);
            AddBuffer<AgentPathPoint>(entity);
            AddComponent<AgentPath>(entity);

            AddComponent<AgenPathRequestedTag>(entity);
            SetComponentEnabled<AgenPathRequestedTag>(entity, false);

            AddComponent<AgentHasEdgePathTag>(entity);
            SetComponentEnabled<AgentHasEdgePathTag>(entity, false);
        }
    }


    public struct NavmeshAgentTag : IComponentData { }

    public struct NavMeshAgent : IComponentData
    {
        public float Radius;
    }

    public struct AgenPathRequestedTag : IComponentData, IEnableableComponent { }

    public struct AgentHasEdgePathTag : IComponentData, IEnableableComponent { }

    public struct AgentDestination : IComponentData
    {
        public float3 Position;
    }

    public struct AgentPathEdge : IBufferElementData
    {
        public float3 PortalVertex1;
        public float3 PortalVertex2;
    }

    public struct AgentPathPoint : IBufferElementData
    {
        public float3 Position;
    }


    public struct AgentPath : IComponentData
    {
        public int PathLength;
        public int PathIndex;
    }
}