using Latios;
using LatiosNavigation.Authoring;
using LatiosNavigation.Utils;
using Unity.Burst;
using Unity.Entities;

namespace LatiosNavigation.Systems
{
    [RequireMatchingQueriesForUpdate]
    public partial struct NavMeshDebugSystem : ISystem
    {
        EntityQuery query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            query = state.Fluent().With<NavMeshSurfaceBlobReference>().Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency = new DebuJgJob().Schedule(query, state.Dependency);
        }

        [BurstCompile]
        partial struct DebuJgJob : IJobEntity
        {
            void Execute(in NavMeshSurfaceBlobReference navMeshSurfaceBlob)
            {
                ref var blobAsset = ref navMeshSurfaceBlob.NavMeshSurfaceBlob.Value;
                NavUtils.Debug(ref blobAsset);
            }
        }
    }
}