using HarmonyLib;
using Model;

namespace Autopilot.Patches
{
    [HarmonyPatch]
    public static class PatchSteamLocomotive
    {
        /// <summary>
        /// Reverse patch to access SteamLocomotive.FuelCar method.
        /// Returns the tender car for a steam locomotive, or the loco itself if it has no tender.
        /// For diesel locos, do NOT call this — use the loco itself.
        /// </summary>
        [HarmonyReversePatch]
        [HarmonyPatch(typeof(SteamLocomotive), "FuelCar")]
        public static Car FuelCar(object instance)
        {
            // Stub — Harmony fills this in at runtime
            throw new System.NotImplementedException("Reverse patch not applied");
        }
    }
}
