using Latios;
using Unity.Entities;

namespace LatiosNavigation.Systems
{
    [DisableAutoCreation]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class NavRootSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            EnableSystemSorting = true;

            GetOrCreateAndAddUnmanagedSystem<AgentPathDebugSystem>();
            GetOrCreateAndAddUnmanagedSystem<AgentEdgePathSystem>();
            GetOrCreateAndAddUnmanagedSystem<AgentPathFunnelingSystem>();
        }
    }
}