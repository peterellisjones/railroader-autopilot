using System.Collections.Generic;
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

        public RouteResult? FindRoute(DirectedPosition from, DirectedPosition to,
            IReadOnlyCollection<Car>? ignoredCars = null, bool checkForCars = true)
        {
            try
            {
                var startLoc = from.ToLocation();
                var endLoc = to.ToLocation();
                if (startLoc.segment == null || endLoc.segment == null)
                    return null;

                var ignored = ignoredCars != null
                    ? new HashSet<Car>(ignoredCars)
                    : new HashSet<Car>();
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

                int reversals = CountReversalsFromSteps(steps);
                return new RouteResult(metrics.Distance, reversals, BlockedByCars: false);
            }
            catch
            {
                return null;
            }
        }

        public RouteResult? FindBestRoute(TrackPosition from, DirectedPosition to,
            IReadOnlyCollection<Car>? ignoredCars = null, bool checkForCars = true)
        {
            // Try both facing directions from the undirected start position
            var facingB = new DirectedPosition(from.Segment, from.DistanceFromA, Direction.TowardEndB);
            var facingA = new DirectedPosition(from.Segment, from.DistanceFromA, Direction.TowardEndA);

            var resultB = FindRoute(facingB, to, ignoredCars, checkForCars);
            var resultA = FindRoute(facingA, to, ignoredCars, checkForCars);

            if (resultB == null) return resultA;
            if (resultA == null) return resultB;
            return resultA.Value.Distance <= resultB.Value.Distance ? resultA : resultB;
        }

        /// <summary>
        /// Count reversals by detecting direction changes in route steps.
        /// Each time the step direction flips (Out→Back or Back→Out) is one reversal.
        /// </summary>
        private static int CountReversalsFromSteps(List<RouteSearch.Step> steps)
        {
            if (steps.Count < 2) return 0;

            int reversals = 0;
            var prevDirection = steps[0].Direction;
            for (int i = 1; i < steps.Count; i++)
            {
                if (steps[i].Direction != prevDirection)
                {
                    reversals++;
                    prevDirection = steps[i].Direction;
                }
            }
            return reversals;
        }
    }
}
