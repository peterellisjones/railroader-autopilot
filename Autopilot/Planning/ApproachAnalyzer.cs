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
            var destLoc = destLocation.ToLocation();

            // Route from the LOCO to the destination — this matches the route
            // the AE will actually take. Previously routed from the tail, which
            // could find a different route with different reversals.
            try
            {
                // Try both loco directions (LocationF and LocationR)
                var stepsF = new List<RouteSearch.Step>();
                bool foundF = RouteSearch.FindRoute(graph, loco.LocationF, destLoc, heuristic,
                    stepsF, out RouteSearch.Metrics metricsF,
                    checkForCars: false, trainLength: 0f,
                    maxIterations: 5000, enableLogging: false);

                var stepsR = new List<RouteSearch.Step>();
                bool foundR = RouteSearch.FindRoute(graph, loco.LocationR, destLoc, heuristic,
                    stepsR, out RouteSearch.Metrics metricsR,
                    checkForCars: false, trainLength: 0f,
                    maxIterations: 5000, enableLogging: false);

                // The AE picks the shorter route
                bool useF = foundF && (!foundR || metricsF.Distance <= metricsR.Distance);
                var bestSteps = useF ? stepsF : stepsR;
                bool found = useF ? foundF : foundR;
                bool locoGoesForward = useF; // LocationF = forward end of loco

                if (!found || bestSteps.Count < 2)
                {
                    _logger.LogDebug("CheckApproach", $"no route to dest={destLocation.Segment?.id}");
                    return true; // assume reachable if no route
                }

                var route = ReversalCounter.DeduplicateSegments(bestSteps);
                _logger.LogApproachDetails("CheckApproach", group, loco, tailOutward, route);

                int reversals = ReversalCounter.FromSegments(route, graph);

                // Determine which side of the loco the tail is on.
                // The tail can be on the forward side (flipped group / pushing)
                // or the rear side (normal / pulling).
                var tailLoc = tailOutward.ToLocation();
                graph.TryFindDistance(loco.LocationF, tailLoc, out float tailDistF, out _);
                graph.TryFindDistance(loco.LocationR, tailLoc, out float tailDistR, out _);
                bool tailOnForwardSide = tailDistF < tailDistR;

                // The AE moves the loco. locoGoesForward + even reversals means
                // the forward end of the loco leads on arrival. Whether the TAIL
                // leads depends on which side of the loco the tail is on.
                //
                // If tail is on forward side: loco forward leads → tail leads
                // If tail is on rear side: loco forward leads → tail trails
                bool oddReversals = (reversals % 2 == 1);
                bool forwardLeadsOnArrival = locoGoesForward ? !oddReversals : oddReversals;
                bool tailLeads = tailOnForwardSide ? forwardLeadsOnArrival : !forwardLeadsOnArrival;

                _logger.LogDebug("CheckApproach", $"locoGoesForward={locoGoesForward}, " +
                    $"tailOnForwardSide={tailOnForwardSide}, reversals={reversals}, " +
                    $"tailLeads={tailLeads} (dest={destLocation.Segment?.id})");

                return tailLeads;
            }
            catch (Exception ex)
            {
                _logger.Log("CheckApproach", $"route search failed: {ex.GetType().Name}: {ex.Message}");
                return false;
            }
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
    }
}
