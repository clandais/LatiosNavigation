using Latios;
using Latios.Authoring;
using LatiosNavigation.Authoring;
using LatiosNavigation.Systems;
using Unity.Entities;

namespace LatiosNavigation
{
    public static class NavBakingBootstrap
    {
        public static void InstallNavBakers(ref CustomBakingBootstrapContext context)
        {
            context.filteredBakerTypes.Add(typeof(NavMeshBaker));
            context.filteredBakerTypes.Add(typeof(NavMeshAgentBaker));
        }
    }

    public static class NavBoostrap
    {
        public static void InstallNav(World world)
        {
            BootstrapTools.InjectSystem(TypeManager.GetSystemTypeIndex<NavRootSystem>(), world);
        }
    }
}