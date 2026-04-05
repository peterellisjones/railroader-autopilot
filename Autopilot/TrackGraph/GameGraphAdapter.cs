using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Patches;

namespace Autopilot.TrackGraph
{
    /// <summary>
    /// Production implementation of IGraphAdapter that wraps Graph.Shared and game types.
    /// Maintains internal dictionaries mapping string IDs to game objects.
    /// </summary>
    public class GameGraphAdapter : IGraphAdapter
    {
        private readonly Track.Graph _graph;

        // Cache: when we encounter game objects, store them for later lookup
        private readonly Dictionary<string, TrackSegment> _segments = new Dictionary<string, TrackSegment>();
        private readonly Dictionary<string, TrackNode> _nodes = new Dictionary<string, TrackNode>();

        public GameGraphAdapter()
        {
            _graph = Track.Graph.Shared;
        }

        /// <summary>Register a segment so it can be looked up by ID later.</summary>
        public void RegisterSegment(TrackSegment seg)
        {
            if (seg != null && !_segments.ContainsKey(seg.id))
                _segments[seg.id] = seg;
        }

        /// <summary>Register a node so it can be looked up by ID later.</summary>
        public void RegisterNode(TrackNode node)
        {
            if (node != null && !_nodes.ContainsKey(node.id))
                _nodes[node.id] = node;
        }

        /// <summary>Get a previously registered segment by ID.</summary>
        public TrackSegment GetSegment(string id) =>
            id != null && _segments.TryGetValue(id, out var s) ? s : null;

        /// <summary>Get a previously registered node by ID.</summary>
        public TrackNode GetNode(string id) =>
            id != null && _nodes.TryGetValue(id, out var n) ? n : null;

        // --- Type conversion methods ---

        /// <summary>Convert a game DirectedPosition to a GraphPosition.</summary>
        public GraphPosition ToGraphPosition(DirectedPosition dp)
            => new(GetSegmentId(dp.Segment), dp.DistanceFromA, dp.Facing);

        /// <summary>Convert a GraphPosition to a game DirectedPosition.</summary>
        public DirectedPosition ToDirectedPosition(GraphPosition gp)
            => new(GetSegment(gp.SegmentId), gp.DistanceFromA, gp.Facing);

        /// <summary>Convert an UndirectedGraphPosition to a game TrackPosition.</summary>
        public TrackPosition ToTrackPosition(UndirectedGraphPosition ugp)
            => new(GetSegment(ugp.SegmentId), ugp.DistanceFromA);

        /// <summary>Resolve car IDs to game Car objects via TrainController.</summary>
        private HashSet<Car> ResolveCarIds(IReadOnlyCollection<string>? carIds)
        {
            var result = new HashSet<Car>();
            if (carIds == null) return result;
            var tc = TrainController.Shared;
            if (tc == null) return result;
            foreach (var id in carIds)
            {
                var car = tc.CarForId(id);
                if (car != null) result.Add(car);
            }
            return result;
        }

        public string GetNodeAtEnd(string segmentId, bool endA)
        {
            var seg = GetSegment(segmentId);
            if (seg == null) return null;
            var node = seg.NodeForEnd(endA ? TrackSegment.End.A : TrackSegment.End.B);
            if (node == null) return null;
            RegisterNode(node);
            return node.id;
        }

        public bool IsEndA(string segmentId, string nodeId)
        {
            var seg = GetSegment(segmentId);
            var node = GetNode(nodeId);
            if (seg == null || node == null) return true;
            return seg.EndForNode(node) == TrackSegment.End.A;
        }

        public string GetOtherNode(string segmentId, string nodeId)
        {
            var seg = GetSegment(segmentId);
            var node = GetNode(nodeId);
            if (seg == null || node == null) return null;
            var other = seg.GetOtherNode(node);
            if (other == null) return null;
            RegisterNode(other);
            return other.id;
        }

        public float GetLength(string segmentId)
        {
            var seg = GetSegment(segmentId);
            return seg?.GetLength() ?? 0f;
        }

        public string GetSegmentId(object segment)
        {
            if (segment is TrackSegment ts)
            {
                RegisterSegment(ts);
                return ts.id;
            }
            return null;
        }

        public bool IsSwitch(string nodeId)
        {
            var node = GetNode(nodeId);
            return node != null && _graph.IsSwitch(node);
        }

        public (string enter, string exitNormal, string exitReverse) GetSwitchLegs(string nodeId)
        {
            var node = GetNode(nodeId);
            if (node == null) return (null, null, null);
            _graph.DecodeSwitchAt(node, out TrackSegment enter, out TrackSegment exitNormal, out TrackSegment exitReverse);
            RegisterSegment(enter);
            RegisterSegment(exitNormal);
            RegisterSegment(exitReverse);
            return (enter?.id, exitNormal?.id, exitReverse?.id);
        }

        public float GetFoulingDistance(string nodeId)
        {
            var node = GetNode(nodeId);
            return node != null ? _graph.CalculateFoulingDistance(node) : 0f;
        }

        public string GetReachableSegment(string segmentId, bool endA)
        {
            var seg = GetSegment(segmentId);
            if (seg == null) return null;
            var end = endA ? TrackSegment.End.A : TrackSegment.End.B;
            GraphPatches.SegmentsReachableFrom(_graph, seg, end, out TrackSegment next, out _);
            if (next == null) return null;
            RegisterSegment(next);
            return next.id;
        }

        private static readonly HeuristicCosts _routeHeuristic = new HeuristicCosts
        {
            DivergingRoute = 100,
            ThrowSwitch = 50,
            ThrowSwitchCTCLocked = 1000,
            CarBlockingRoute = 500
        };

        public RouteResult? FindRoute(GraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true)
        {
            try
            {
                var fromDp = ToDirectedPosition(from);
                var toDp = ToDirectedPosition(to);
                var startLoc = fromDp.ToLocation();
                var endLoc = toDp.ToLocation();
                if (startLoc.segment == null || endLoc.segment == null)
                    return null;

                var ignored = ResolveCarIds(ignoredCarIds);
                var impasse = new HashSet<Car>();
                var steps = new List<RouteSearch.Step>();

                bool found = RouteSearch.FindRoute(
                    _graph, startLoc, endLoc, _routeHeuristic,
                    steps, out RouteSearch.Metrics metrics,
                    checkForCars: checkForCars, trainLength: 0f,
                    trainMomentum: 0f, maxIterations: 5000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: impasse);

                if (!found)
                {
                    // If blocked by cars (impasse non-empty), report that
                    if (impasse.Count > 0)
                        return new RouteResult(float.MaxValue, 0, BlockedByCars: true);
                    return null;
                }

                var deduped = Planning.ReversalCounter.DeduplicateSegments(steps);
                int reversals = Planning.ReversalCounter.FromSegments(deduped, _graph);
                var segIds = deduped.ConvertAll(s => s.id);
                return new RouteResult(metrics.Distance, reversals, BlockedByCars: false, segIds);
            }
            catch
            {
                return null;
            }
        }

        public RouteResult? FindBestRoute(UndirectedGraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true)
        {
            // Try both facing directions from the undirected start position
            var facingB = new GraphPosition(from.SegmentId, from.DistanceFromA, Direction.TowardEndB);
            var facingA = new GraphPosition(from.SegmentId, from.DistanceFromA, Direction.TowardEndA);

            var resultB = FindRoute(facingB, to, ignoredCarIds, checkForCars);
            var resultA = FindRoute(facingA, to, ignoredCarIds, checkForCars);

            if (resultB == null) return resultA;
            if (resultA == null) return resultB;
            return resultA.Value.Distance <= resultB.Value.Distance ? resultA : resultB;
        }

        public string? FindSharedNode(string segmentIdA, string segmentIdB)
        {
            var nodeAA = GetNodeAtEnd(segmentIdA, true);
            var nodeAB = GetNodeAtEnd(segmentIdA, false);
            var nodeBA = GetNodeAtEnd(segmentIdB, true);
            var nodeBB = GetNodeAtEnd(segmentIdB, false);
            if (nodeAA != null && (nodeAA == nodeBA || nodeAA == nodeBB)) return nodeAA;
            if (nodeAB != null && (nodeAB == nodeBA || nodeAB == nodeBB)) return nodeAB;
            return null;
        }

        public bool? DirectionTowardIsEndA(string fromSegmentId, string toSegmentId, int maxHops = 5)
        {
            if (fromSegmentId == toSegmentId) return null;

            // Try walking from End A
            if (WalkToward(fromSegmentId, true, toSegmentId, maxHops))
                return true;

            // Try walking from End B
            if (WalkToward(fromSegmentId, false, toSegmentId, maxHops))
                return false;

            return null;
        }

        /// <summary>
        /// Walk from the given end of a segment, crossing nodes, checking if toSegmentId is reachable.
        /// </summary>
        private bool WalkToward(string startSegId, bool startEndA, string targetSegId, int maxHops)
        {
            var currentSegId = startSegId;
            var currentEndA = startEndA;

            for (int hop = 0; hop < maxHops; hop++)
            {
                var nodeId = GetNodeAtEnd(currentSegId, currentEndA);
                if (nodeId == null) return false;

                if (IsSwitch(nodeId))
                {
                    // At a switch, check all legs
                    var (enter, exitNormal, exitReverse) = GetSwitchLegs(nodeId);
                    var candidates = new[] { enter, exitNormal, exitReverse }
                        .Where(s => s != null && s != currentSegId);
                    foreach (var nextSegId in candidates)
                    {
                        if (nextSegId == targetSegId) return true;
                    }
                    // Don't continue through switches — would require branching search
                    return false;
                }
                else
                {
                    // Non-switch node: continue to the next segment
                    var nextSegId = GetReachableSegment(currentSegId, currentEndA);
                    if (nextSegId == null) return false;
                    if (nextSegId == targetSegId) return true;

                    // Continue walking: determine which end of nextSeg the node is at
                    var nextNodeA = GetNodeAtEnd(nextSegId, true);
                    // If nodeId is at End A of nextSeg, we continue from End B (the far end)
                    currentEndA = nextNodeA != nodeId;
                    currentSegId = nextSegId;
                }
            }
            return false;
        }
    }
}
