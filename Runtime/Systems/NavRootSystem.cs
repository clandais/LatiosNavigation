using Latios;
using Latios.Transforms.Systems;
using Unity.Entities;

namespace LatiosNavigation.Systems
{
    [DisableAutoCreation]
    [UpdateInGroup(typeof(PreTransformSuperSystem))]
    public partial class NavRootSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            EnableSystemSorting = true;
            GetOrCreateAndAddUnmanagedSystem<AgentEdgePathSystem>();
            GetOrCreateAndAddUnmanagedSystem<AgentPathFunnelingSystem>();
        }
    }
}