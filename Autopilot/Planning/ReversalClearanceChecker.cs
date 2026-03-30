using System.Collections.Generic;
using Track;

namespace Autopilot.Planning
{
    /// <summary>
    /// Walks the track graph away from a switch node to find the maximum
    /// reachable distance. Used to check whether the loco has enough space
    /// to reverse at a loop switch.
    /// </summary>
    public static class ReversalClearanceChecker
    {
        public const int MaxRecursionDepth = 10;

        /// <summary>
        /// Find the maximum reachable distance walking away from a switch node
        /// along the given segment. At switches, explores all branches (DFS)
        /// and returns the best. Stops at dead ends, blocked segments, or depth limit.
        /// </summary>
        public static float WalkMaxDistance(TrackSegment startSeg, TrackNode fromNode,
            float target, HashSet<string> blockedSegIds)
        {
            var graph = Graph.Shared;
            // The start segment is the stem — always included (train is already here)
            return startSeg.GetLength() + WalkMaxDistanceRecursive(
                graph, startSeg, fromNode, target - startSeg.GetLength(), 0, blockedSegIds);
        }

        private static float WalkMaxDistanceRecursive(
            Graph graph, TrackSegment seg, TrackNode entryNode, float remaining, int depth,
            HashSet<string> blockedSegIds)
        {
            if (remaining <= 0f || depth >= MaxRecursionDepth)
                return 0f;

            var farNode = seg.GetOtherNode(entryNode);
            if (farNode == null)
                return 0f; // dead end

            if (graph.IsSwitch(farNode))
            {
                graph.DecodeSwitchAt(farNode, out TrackSegment enter,
                    out TrackSegment exitN, out TrackSegment exitR);

                float best = 0f;
                foreach (var next in new[] { enter, exitN, exitR })
                {
                    if (next == null || next == seg) continue;
                    if (blockedSegIds.Contains(next.id)) continue; // blocked by car
                    float len = next.GetLength();
                    float branchTotal = len + WalkMaxDistanceRecursive(
                        graph, next, farNode, remaining - len, depth + 1, blockedSegIds);
                    if (branchTotal > best) best = branchTotal;
                }
                return best;
            }
            else
            {
                var farEnd = seg.EndForNode(farNode);
                Patches.GraphPatches.SegmentsReachableFrom(graph, seg, farEnd,
                    out TrackSegment nextSeg, out _);
                if (nextSeg == null)
                    return 0f;
                if (blockedSegIds.Contains(nextSeg.id))
                    return 0f; // blocked by car
                float len = nextSeg.GetLength();
                return len + WalkMaxDistanceRecursive(
                    graph, nextSeg, farNode, remaining - len, depth + 1, blockedSegIds);
            }
        }
    }
}
