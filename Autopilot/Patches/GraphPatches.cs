using HarmonyLib;
using Track;

namespace Autopilot.Patches
{
    [HarmonyPatch]
    public static class GraphPatches
    {
        [HarmonyReversePatch]
        [HarmonyPatch(typeof(Graph), "SegmentsReachableFrom")]
        public static void SegmentsReachableFrom(
            Graph instance,
            TrackSegment segment,
            TrackSegment.End end,
            out TrackSegment normal,
            out TrackSegment reversed)
        {
            // Stub — Harmony fills this in at runtime
            throw new System.NotImplementedException("Reverse patch not applied");
        }
    }
}
