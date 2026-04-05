using System;
using System.Collections.Generic;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.TrackGraph;

namespace Autopilot.Planning
{
    public class ApproachAnalyzer
    {
        public const int MaxSpanWalkSegments = 10;

        private readonly RouteChecker _routeChecker;
        private readonly PlanningLogger _logger;

        // Testable planning fields (used by interface-based constructor path)
        private readonly IGraphAdapter? _iGraph;
        private readonly IPlanningLogger? _iLogger;

        public ApproachAnalyzer(RouteChecker routeChecker, PlanningLogger logger)
        {
            _routeChecker = routeChecker;
            _logger = logger;
        }

        /// <summary>Constructor for testable planning.</summary>
        public ApproachAnalyzer(RouteChecker routeChecker, IGraphAdapter graph, IPlanningLogger logger)
        {
            _routeChecker = routeChecker;
            _iGraph = graph;
            _iLogger = logger;
            _logger = null!;
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
        public bool CheckApproachDirection(BaseLocomotive loco, CarGroup group, SpanBoundary destLocation)
        {
            var graph = Graph.Shared;
            var heuristic = new HeuristicCosts
            {
                DivergingRoute = 100,
                ThrowSwitch = 50,
                ThrowSwitchCTCLocked = 1000,
                CarBlockingRoute = 500
            };

            var tailOutwardGp = group.TailOutwardEnd!.Value;
            var tailInwardGp = group.TailInwardEnd!.Value;

            // Convert GraphPosition back to DirectedPosition for game route search
            var adapter = new GameGraphAdapter();
            RegisterSegmentById(adapter, tailOutwardGp.SegmentId);
            RegisterSegmentById(adapter, tailInwardGp.SegmentId);
            RegisterSegmentById(adapter, destLocation.SegmentId);

            var tailOutward = adapter.ToDirectedPosition(tailOutwardGp);
            var tailInward = adapter.ToDirectedPosition(tailInwardGp);
            var destLoc = destLocation.ToLocation(adapter);

            int reversals = 0;
            bool tailFacesDest = true;

            if (tailOutward.Segment != null)
            {
                try
                {
                    var tailOutwardLoc = tailOutward.ToLocation();

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

                    bool useOutward = outwardFound && (!flippedFound || outwardMetrics.Distance <= flippedMetrics.Distance);
                    var tailRouteSteps = useOutward ? outwardSteps : flippedSteps;
                    bool found = useOutward ? outwardFound : flippedFound;

                    if (found && tailRouteSteps.Count >= 2)
                    {
                        var route = ReversalCounter.DeduplicateSegments(tailRouteSteps);

                        _logger.LogApproachDetails("CheckApproach", group, loco, tailOutward, route);

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

                        reversals = ReversalCounter.FromSegments(route, graph);
                    }
                }
                catch (Exception ex)
                {
                    _logger.Log("CheckApproach", $"route search failed: {ex.GetType().Name}: {ex.Message}");
                    return false;
                }
            }

            bool oddReversals = (reversals % 2 == 1);
            bool tailLeads = tailFacesDest != oddReversals;

            _logger.LogDebug("CheckApproach", $"tailFacesDest={tailFacesDest}, " +
                $"reversals={reversals}, tailLeads={tailLeads} (dest={destLocation.SegmentId})");

            return tailLeads;
        }

        private static void RegisterSegmentById(GameGraphAdapter adapter, string segmentId)
        {
            if (segmentId == null) return;
            var graph = Track.Graph.Shared;
            foreach (var seg in graph.Segments)
            {
                if (seg.id == segmentId)
                {
                    adapter.RegisterSegment(seg);
                    return;
                }
            }
        }

        /// <summary>
        /// Check if any car in the consist is within a destination span.
        /// Walks the graph from destLocation's segment outward (up to MaxSpanWalkSegments segments)
        /// and checks if any consist segment is on the path. This handles
        /// multi-segment spans where the train is between the endpoint segments.
        /// </summary>
        public static bool IsConsistWithinSpan(BaseLocomotive loco, CarGroup group, SpanBoundary destLocation)
        {
            if (destLocation.SegmentId == null) return false;

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
            if (consistSegIds.Contains(destLocation.SegmentId)) return true;

            // Walk outward from the destination segment in both directions.
            // Resolve the segment ID to a game TrackSegment for walking.
            var graph = Graph.Shared;
            TrackSegment? destSeg = null;
            foreach (var s in graph.Segments)
            {
                if (s.id == destLocation.SegmentId) { destSeg = s; break; }
            }
            if (destSeg == null) return false;

            foreach (var startEnd in new[] { TrackSegment.End.A, TrackSegment.End.B })
            {
                var seg = destSeg;
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

        // =================================================================
        // Testable overloads using IGraphAdapter + GraphPosition
        // =================================================================

        /// <summary>
        /// Check approach direction with explicit tail positions (GraphPosition).
        /// This is the fully abstract version — no game types needed.
        /// tailLeads = tailFacesDest XOR oddReversals
        /// </summary>
        public bool CheckApproachDirection(GraphPosition tailOutward, GraphPosition tailInward,
            GraphPosition destLocation)
        {
            int reversals = 0;
            bool tailFacesDest = true;

            if (tailOutward.SegmentId != null)
            {
                // Try both directions from tail position (outward and flipped)
                var outwardResult = _iGraph!.FindRoute(tailOutward, destLocation, checkForCars: false);
                var flippedResult = _iGraph.FindRoute(tailOutward.Flipped, destLocation, checkForCars: false);

                bool useOutward = outwardResult != null
                    && (flippedResult == null || outwardResult.Value.Distance <= flippedResult.Value.Distance);
                var bestResult = useOutward ? outwardResult : flippedResult;

                if (bestResult != null && bestResult.Value.RouteSegmentIds != null
                    && bestResult.Value.RouteSegmentIds.Count >= 2)
                {
                    var route = bestResult.Value.RouteSegmentIds;

                    // Determine if route exits outward from the tail
                    tailFacesDest = RouteGoesOutward(tailOutward, tailInward, route);

                    reversals = bestResult.Value.ReversalCount;
                }
            }

            bool oddReversals = (reversals % 2 == 1);
            bool tailLeads = tailFacesDest != oddReversals;

            _iLogger?.LogDebug("CheckApproach", $"tailFacesDest={tailFacesDest}, " +
                $"reversals={reversals}, tailLeads={tailLeads} (dest={destLocation.SegmentId})");

            return tailLeads;
        }

        /// <summary>
        /// Determine if a route exits the starting segment in the outward direction.
        /// Uses IGraphAdapter for node lookups instead of game types.
        /// </summary>
        private bool RouteGoesOutward(GraphPosition outward, GraphPosition inward,
            IReadOnlyList<string> routeSegments)
        {
            if (routeSegments == null || routeSegments.Count < 2) return true;

            // Find the node between the first two segments of the route
            var exitNode = _iGraph!.FindSharedNode(routeSegments[0], routeSegments[1]);
            if (exitNode == null) return true;

            if (outward.SegmentId == inward.SegmentId)
            {
                // Same segment: outward end is farther from inward
                bool outwardIsEndB = outward.DistanceFromA > inward.DistanceFromA;
                // Route exits at End A of the segment if the exit node is at End A
                bool routeExitsEndA = _iGraph.IsEndA(routeSegments[0], exitNode);
                // Route goes outward if it exits toward the outward end
                return outwardIsEndB ? !routeExitsEndA : routeExitsEndA;
            }
            else
            {
                // Different segments: find connection between outward and inward segments
                var connectNode = _iGraph.FindSharedNode(outward.SegmentId, inward.SegmentId);
                if (connectNode != null)
                {
                    bool connectIsEndA = _iGraph.IsEndA(outward.SegmentId, connectNode);
                    bool routeExitsEndA = _iGraph.IsEndA(routeSegments[0], exitNode);
                    // Outward direction is AWAY from the connection
                    return routeExitsEndA != connectIsEndA;
                }
            }

            return true; // default
        }

        // --- Private helpers ---
    }
}
