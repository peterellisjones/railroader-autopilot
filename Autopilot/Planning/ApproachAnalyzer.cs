using System;
using System.Collections.Generic;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class ApproachAnalyzer
    {
        public const int MaxSpanWalkSegments = 10;

        private readonly RouteChecker _routeChecker;
        private readonly PlanningLogger _logger;

        public ApproachAnalyzer(RouteChecker routeChecker, PlanningLogger logger)
        {
            _routeChecker = routeChecker;
            _logger = logger;
        }

        /// <summary>
        /// Check if the tail leads into the destination siding.
        ///
        /// Route from the tail's outward end to the destination. Count
        /// exit-to-exit switch traversals (reversals). Even = tail leads
        /// (orientation preserved). Odd = loco leads (orientation flipped).
        ///
        /// For reversed groups: the Reversed() method already swaps the
        /// outward/inward ends, so tailFacesDest and tailLeads are computed
        /// from the correct post-runaround position.
        /// </summary>
        public bool CheckApproachDirection(BaseLocomotive loco, CarGroup group, DirectedPosition destLocation)
        {
            var graph = Graph.Shared;
            var heuristic = new HeuristicCosts
            {
                DivergingRoute = 100,
                ThrowSwitch = 50,
                ThrowSwitchCTCLocked = 1000,
                CarBlockingRoute = 500
            };

            var tailOutward = group.TailOutwardEnd!.Value;
            var tailInward = group.TailInwardEnd!.Value;
            var destLoc = destLocation.ToLocation();

            // Route from tail to destination. RouteSearch exits the starting
            // segment in one direction only (the facing direction of the Location).
            // tailOutward and tailOutward.Flipped face OPPOSITE directions on
            // the same segment, so we try both and pick the shorter route.
            int reversals = 0;
            bool tailFacesDest = true; // default if no route or single segment

            if (tailOutward.Segment != null)
            {
                try
                {
                    var tailOutwardLoc = tailOutward.ToLocation();

                    // Try both directions from the tail's position
                    var outwardSteps = new List<RouteSearch.Step>();
                    bool outwardFound = RouteSearch.FindRoute(graph, tailOutwardLoc, destLoc, heuristic,
                        outwardSteps, out RouteSearch.Metrics outwardMetrics,
                        checkForCars: false, trainLength: 0f,
                        maxIterations: 5000, enableLogging: false);

                    var flippedLoc = tailOutward.Flipped.ToLocation();
                    var flippedSteps = new List<RouteSearch.Step>();
                    bool flippedFound = RouteSearch.FindRoute(graph, flippedLoc, destLoc, heuristic,
                        flippedSteps, out RouteSearch.Metrics flippedMetrics,
                        checkForCars: false, trainLength: 0f,
                        maxIterations: 5000, enableLogging: false);

                    // Pick the shorter route — it represents the actual approach
                    bool useOutward = outwardFound && (!flippedFound || outwardMetrics.Distance <= flippedMetrics.Distance);
                    var tailRouteSteps = useOutward ? outwardSteps : flippedSteps;
                    bool found = useOutward ? outwardFound : flippedFound;

                    if (found && tailRouteSteps.Count >= 2)
                    {
                        var route = DeduplicateRouteSegments(tailRouteSteps);

                        _logger.LogApproachDetails("CheckApproach", group, loco, tailOutward, route);

                        // #2: Which end of the tail car is closer to the first switch
                        // on the route? If the free (outward) end → train goes forward
                        // → need even reversals. If the coupled (inward) end → train
                        // goes backward → need odd reversals.
                        TrackNode firstRouteSwitch = null;
                        for (int j = 0; j < route.Count - 1; j++)
                        {
                            var switchNode = Services.TrackWalker.FindSharedNode(route[j], route[j + 1]);
                            if (switchNode != null && graph.IsSwitch(switchNode))
                            {
                                firstRouteSwitch = switchNode;
                                break;
                            }
                        }

                        // Check both methods for tailFacesDest
                        bool routeGoesOutward = Services.TrackWalker.RouteGoesOutward(
                            tailOutward, tailInward, route);

                        if (firstRouteSwitch != null)
                        {
                            var tailOutwardLoc2 = tailOutward.ToLocation();
                            var inwardLoc = tailInward.ToLocation();
                            var switchSeg = route[0];
                            var switchEnd = switchSeg.EndForNode(firstRouteSwitch);
                            var switchLoc = new Location(switchSeg, 0f, switchEnd);

                            graph.TryFindDistance(tailOutwardLoc2, switchLoc, out float distOutward, out _);
                            graph.TryFindDistance(inwardLoc, switchLoc, out float distInward, out _);

                            bool distanceCheck = distOutward < distInward;

                            // Use RouteGoesOutward as the primary check.
                            // It directly checks which end of the tail's segment
                            // the route exits from — no ambiguity with distances.
                            tailFacesDest = routeGoesOutward;

                            _logger.LogDebug("CheckApproach", $"firstSwitch={firstRouteSwitch.id}, " +
                                $"RouteGoesOutward={routeGoesOutward}, distCheck={distanceCheck}, " +
                                $"distOutward={distOutward:F0}, distInward={distInward:F0}, " +
                                $"tailFacesDest={tailFacesDest}");
                        }
                        else
                        {
                            tailFacesDest = routeGoesOutward;
                            _logger.LogDebug("CheckApproach", $"no switch, RouteGoesOutward={routeGoesOutward}, " +
                                $"tailFacesDest={tailFacesDest}");
                        }

                        reversals = CountReversals(route, graph);
                    }
                }
                catch (Exception ex)
                {
                    _logger.Log("CheckApproach", $"route search failed: {ex.GetType().Name}: {ex.Message}");
                    return false;
                }
            }

            // Combine: tailFacesDest XOR oddReversals
            bool oddReversals = (reversals % 2 == 1);
            bool tailLeads = tailFacesDest != oddReversals;

            _logger.LogDebug("CheckApproach", $"tailFacesDest={tailFacesDest}, " +
                $"reversals={reversals}, tailLeads={tailLeads} (dest={destLocation.Segment?.id})");

            return tailLeads;
        }

        /// <summary>
        /// Check if any car in the consist is within a destination span.
        /// Walks the graph from destLocation's segment outward (up to MaxSpanWalkSegments segments)
        /// and checks if any consist segment is on the path. This handles
        /// multi-segment spans where the train is between the endpoint segments.
        /// </summary>
        public static bool IsConsistWithinSpan(BaseLocomotive loco, CarGroup group, DirectedPosition destLocation)
        {
            if (destLocation.Segment == null) return false;

            // Collect all segments the consist occupies
            var consistSegIds = new HashSet<string>();
            if (loco.LocationF.segment != null) consistSegIds.Add(loco.LocationF.segment.id);
            if (loco.LocationR.segment != null) consistSegIds.Add(loco.LocationR.segment.id);
            foreach (var car in group.Cars)
            {
                if (car.EndA.Segment != null) consistSegIds.Add(car.EndA.Segment.id);
                if (car.EndB.Segment != null) consistSegIds.Add(car.EndB.Segment.id);
            }

            // Direct match
            if (consistSegIds.Contains(destLocation.Segment.id)) return true;

            // Walk outward from the destination segment in both directions
            // (up to MaxSpanWalkSegments segments each way) looking for a consist segment.
            // This finds the train within a multi-segment span.
            var graph = Graph.Shared;
            foreach (var startEnd in new[] { TrackSegment.End.A, TrackSegment.End.B })
            {
                var seg = destLocation.Segment;
                var walkEnd = startEnd;
                for (int i = 0; i < MaxSpanWalkSegments; i++)
                {
                    var node = seg.NodeForEnd(walkEnd);
                    if (node == null) break;

                    TrackSegment nextSeg = null;
                    if (graph.IsSwitch(node))
                    {
                        // At a switch: try all legs
                        graph.DecodeSwitchAt(node, out TrackSegment enter,
                            out TrackSegment exitN, out TrackSegment exitR);
                        foreach (var candidate in new[] { enter, exitN, exitR })
                        {
                            if (candidate != null && candidate != seg)
                            {
                                if (consistSegIds.Contains(candidate.id)) return true;
                                // Pick the first non-visited leg to continue walking
                                if (nextSeg == null) nextSeg = candidate;
                            }
                        }
                    }
                    else
                    {
                        Patches.GraphPatches.SegmentsReachableFrom(graph, seg, walkEnd,
                            out nextSeg, out _);
                        if (nextSeg != null && consistSegIds.Contains(nextSeg.id))
                            return true;
                    }

                    if (nextSeg == null) break;
                    var nextNode = nextSeg.GetOtherNode(node);
                    if (nextNode == null) break;
                    walkEnd = nextSeg.EndForNode(nextNode);
                    seg = nextSeg;
                }
            }

            return false;
        }

        // --- Private helpers ---

        /// <summary>
        /// Deduplicate consecutive identical segments from RouteSearch steps.
        /// Logs a warning for non-consecutive duplicate segments.
        /// </summary>
        private List<TrackSegment> DeduplicateRouteSegments(List<RouteSearch.Step> routeSteps)
        {
            var route = new List<TrackSegment>();
            var seenSegIds = new HashSet<string>();
            foreach (var step in routeSteps)
            {
                if (step.Location.segment == null) continue;
                if (route.Count > 0 && route[route.Count - 1] == step.Location.segment)
                    continue; // consecutive duplicate

                if (!seenSegIds.Add(step.Location.segment.id))
                    _logger.LogDebug("CheckApproach", $"WARNING — non-consecutive duplicate segment {step.Location.segment.id}");

                route.Add(step.Location.segment);
            }
            return route;
        }

        /// <summary>
        /// Count reversals in a deduplicated route segment list.
        /// A reversal is either: a switch visited more than once (extra visits = extra reversals),
        /// or an exit→exit transition at a switch visited exactly once (compressed reversal).
        /// </summary>
        private int CountReversals(List<TrackSegment> route, Graph graph)
        {
            int reversals = 0;

            // Count switch visit frequencies and log leg transitions
            var switchVisits = new Dictionary<string, int>();
            for (int j = 0; j < route.Count - 1; j++)
            {
                var node = Services.TrackWalker.FindSharedNode(route[j], route[j + 1]);
                if (node == null || !graph.IsSwitch(node))
                    continue;

                if (!switchVisits.ContainsKey(node.id))
                    switchVisits[node.id] = 0;
                switchVisits[node.id]++;

                graph.DecodeSwitchAt(node, out TrackSegment enter,
                    out TrackSegment exitNormal, out TrackSegment exitReverse);

                string fromLeg = route[j] == enter ? "enter" : (route[j] == exitNormal ? "exitN" : "exitR");
                string toLeg = route[j + 1] == enter ? "enter" : (route[j + 1] == exitNormal ? "exitN" : "exitR");
                _logger.LogDebug("CheckApproach", $"switch {node.id}: {fromLeg}→{toLeg}");
            }

            // Each switch visited N times has N-1 reversals at that switch
            foreach (var kvp in switchVisits)
            {
                int extraVisits = kvp.Value - 1;
                if (extraVisits > 0)
                {
                    reversals += extraVisits;
                    _logger.LogDebug("CheckApproach", $"switch {kvp.Key} visited {kvp.Value}x → {extraVisits} reversal(s)");
                }
            }

            // Also count exit→exit transitions (compressed reversals)
            for (int j = 0; j < route.Count - 1; j++)
            {
                var node = Services.TrackWalker.FindSharedNode(route[j], route[j + 1]);
                if (node == null || !graph.IsSwitch(node))
                    continue;

                graph.DecodeSwitchAt(node, out TrackSegment enter,
                    out TrackSegment exitNormal, out TrackSegment exitReverse);

                bool fromIsExit = (route[j] == exitNormal || route[j] == exitReverse);
                bool toIsExit = (route[j + 1] == exitNormal || route[j + 1] == exitReverse);

                // Only count if this switch was visited once (not already counted above)
                if (fromIsExit && toIsExit && switchVisits[node.id] == 1)
                {
                    reversals++;
                    _logger.LogDebug("CheckApproach", $"exit→exit REVERSAL at {node.id}");
                }
            }

            return reversals;
        }
    }
}
