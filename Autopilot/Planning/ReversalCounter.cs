using System.Collections.Generic;
using Track;
using Track.Search;
using Autopilot.TrackGraph;

namespace Autopilot.Planning
{
    /// <summary>
    /// Single source of truth for counting reversals in a route.
    /// Used by both ApproachAnalyzer and LoopValidator — must stay
    /// consistent so reversal parity checks are reliable.
    ///
    /// A reversal is either:
    ///   1. A switch visited more than once (N visits = N-1 reversals)
    ///   2. An exit→exit transition at a switch visited exactly once
    ///      (compressed reversal — RouteSearch optimized out the backtrack)
    /// </summary>
    public static class ReversalCounter
    {
        /// <summary>
        /// Count reversals from raw RouteSearch steps.
        /// Deduplicates consecutive segments, then counts.
        /// </summary>
        public static int FromSteps(List<RouteSearch.Step> steps)
        {
            var route = DeduplicateSegments(steps);
            return FromSegments(route, Graph.Shared);
        }

        /// <summary>
        /// Count reversals from a deduplicated segment list.
        /// </summary>
        public static int FromSegments(List<TrackSegment> route, Graph graph)
        {
            int reversals = 0;

            var switchVisits = new Dictionary<string, int>();
            for (int j = 0; j < route.Count - 1; j++)
            {
                var node = Services.TrackWalker.FindSharedNode(route[j], route[j + 1]);
                if (node == null || !graph.IsSwitch(node))
                    continue;

                if (!switchVisits.ContainsKey(node.id))
                    switchVisits[node.id] = 0;
                switchVisits[node.id]++;
            }

            // Each switch visited N times has N-1 reversals
            foreach (var kvp in switchVisits)
            {
                if (kvp.Value > 1)
                    reversals += kvp.Value - 1;
            }

            // Exit→exit at switches visited exactly once (compressed reversals)
            for (int j = 0; j < route.Count - 1; j++)
            {
                var node = Services.TrackWalker.FindSharedNode(route[j], route[j + 1]);
                if (node == null || !graph.IsSwitch(node))
                    continue;

                graph.DecodeSwitchAt(node, out var enter, out var exitN, out var exitR);
                bool fromIsExit = (route[j] == exitN || route[j] == exitR);
                bool toIsExit = (route[j + 1] == exitN || route[j + 1] == exitR);

                if (fromIsExit && toIsExit && switchVisits[node.id] == 1)
                    reversals++;
            }

            return reversals;
        }

        /// <summary>
        /// Count reversals from a list of segment IDs using IGraphAdapter.
        /// A switch visited N times contributes N-1 reversals.
        /// </summary>
        public static int FromSegmentIds(IReadOnlyList<string> route, IGraphAdapter graph)
        {
            if (route.Count < 3) return 0;
            int reversals = 0;
            var switchVisits = new Dictionary<string, int>();

            for (int i = 0; i < route.Count - 1; i++)
            {
                var sharedNode = graph.FindSharedNode(route[i], route[i + 1]);
                if (sharedNode != null && graph.IsSwitch(sharedNode))
                {
                    if (switchVisits.ContainsKey(sharedNode))
                        switchVisits[sharedNode]++;
                    else
                        switchVisits[sharedNode] = 1;
                }
            }

            foreach (var kv in switchVisits)
            {
                if (kv.Value > 1) reversals += kv.Value - 1;
            }
            return reversals;
        }

        /// <summary>
        /// Deduplicate consecutive identical segments from RouteSearch steps.
        /// </summary>
        public static List<TrackSegment> DeduplicateSegments(List<RouteSearch.Step> steps)
        {
            var route = new List<TrackSegment>();
            foreach (var step in steps)
            {
                if (step.Location.segment == null) continue;
                if (route.Count > 0 && route[route.Count - 1] == step.Location.segment)
                    continue;
                route.Add(step.Location.segment);
            }
            return route;
        }
    }
}
