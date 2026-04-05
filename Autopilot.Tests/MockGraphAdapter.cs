using System;
using System.Collections.Generic;
using System.Linq;
using Autopilot.Model;
using Autopilot.TrackGraph;

namespace Autopilot.Tests
{
    /// <summary>
    /// Mock graph adapter for testing LoopFinder with configurable track topologies.
    /// Build a graph by adding segments and switches, then pass to LoopFinder.
    /// </summary>
    public class MockGraphAdapter : IGraphAdapter
    {
        private readonly Dictionary<string, SegmentDef> _segments = new Dictionary<string, SegmentDef>();
        private readonly Dictionary<string, SwitchDef> _switches = new Dictionary<string, SwitchDef>();
        private readonly Dictionary<string, RouteResult> _registeredRoutes = new Dictionary<string, RouteResult>();
        private readonly HashSet<string> _blockedCarSegments = new HashSet<string>();

        private class SegmentDef
        {
            public string Id;
            public string NodeA; // node at End.A
            public string NodeB; // node at End.B
            public float Length;
        }

        private class SwitchDef
        {
            public string Enter;       // segment ID
            public string ExitNormal;  // segment ID
            public string ExitReverse; // segment ID
            public float Fouling;
        }

        // --- Builder API ---

        /// <summary>
        /// Add a segment between two nodes.
        /// </summary>
        public MockGraphAdapter AddSegment(string segId, string nodeA, string nodeB, float length)
        {
            _segments[segId] = new SegmentDef { Id = segId, NodeA = nodeA, NodeB = nodeB, Length = length };
            return this;
        }

        /// <summary>
        /// Declare a node as a switch with its three legs.
        /// The enter/exitNormal/exitReverse are segment IDs.
        /// </summary>
        public MockGraphAdapter AddSwitch(string nodeId, string enter, string exitNormal, string exitReverse, float fouling = 10f)
        {
            _switches[nodeId] = new SwitchDef
            {
                Enter = enter,
                ExitNormal = exitNormal,
                ExitReverse = exitReverse,
                Fouling = fouling
            };
            return this;
        }

        /// <summary>
        /// Pre-register a route result for a specific from→to segment pair.
        /// When FindRoute is called with matching segment IDs, returns this result
        /// without running BFS.
        /// </summary>
        public MockGraphAdapter RegisterRoute(GraphPosition from, GraphPosition to, RouteResult result)
        {
            _registeredRoutes[$"{from.SegmentId}\u2192{to.SegmentId}"] = result;
            return this;
        }

        /// <summary>
        /// Mark a segment as blocked by an uncoupled car.
        /// When checkForCars is true, BFS will not traverse blocked segments
        /// (unless the segment ID appears in ignoredCarIds).
        /// </summary>
        public MockGraphAdapter BlockSegment(string segmentId)
        {
            _blockedCarSegments.Add(segmentId);
            return this;
        }

        // --- IGraphAdapter ---

        public string GetNodeAtEnd(string segmentId, bool endA)
        {
            if (!_segments.TryGetValue(segmentId, out var seg)) return null;
            return endA ? seg.NodeA : seg.NodeB;
        }

        public bool IsEndA(string segmentId, string nodeId)
        {
            if (!_segments.TryGetValue(segmentId, out var seg)) return true;
            return seg.NodeA == nodeId;
        }

        public string GetOtherNode(string segmentId, string nodeId)
        {
            if (!_segments.TryGetValue(segmentId, out var seg)) return null;
            if (seg.NodeA == nodeId) return seg.NodeB;
            if (seg.NodeB == nodeId) return seg.NodeA;
            return null;
        }

        public float GetLength(string segmentId)
        {
            return _segments.TryGetValue(segmentId, out var seg) ? seg.Length : 0f;
        }

        public string GetSegmentId(object segment)
        {
            return segment as string;
        }

        public bool IsSwitch(string nodeId)
        {
            return _switches.ContainsKey(nodeId);
        }

        public (string enter, string exitNormal, string exitReverse) GetSwitchLegs(string nodeId)
        {
            if (!_switches.TryGetValue(nodeId, out var sw)) return (null, null, null);
            return (sw.Enter, sw.ExitNormal, sw.ExitReverse);
        }

        public float GetFoulingDistance(string nodeId)
        {
            return _switches.TryGetValue(nodeId, out var sw) ? sw.Fouling : 0f;
        }

        public string GetReachableSegment(string segmentId, bool endA)
        {
            // For non-switch nodes: find the other segment connected to the same node
            var nodeId = GetNodeAtEnd(segmentId, endA);
            if (nodeId == null || IsSwitch(nodeId)) return null;

            foreach (var kv in _segments)
            {
                if (kv.Key == segmentId) continue;
                if (kv.Value.NodeA == nodeId || kv.Value.NodeB == nodeId)
                    return kv.Key;
            }
            return null;
        }

        public RouteResult? FindRoute(GraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true)
        {
            // Check registered routes first
            var key = $"{from.SegmentId}\u2192{to.SegmentId}";
            if (_registeredRoutes.TryGetValue(key, out var registered))
                return registered;

            // BFS over mock topology
            return BfsRoute(from, to, checkForCars, ignoredCarIds);
        }

        public RouteResult? FindBestRoute(UndirectedGraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true)
        {
            var fwd = FindRoute(
                new GraphPosition(from.SegmentId, from.DistanceFromA, Direction.TowardEndB),
                to, ignoredCarIds, checkForCars);
            var bwd = FindRoute(
                new GraphPosition(from.SegmentId, from.DistanceFromA, Direction.TowardEndA),
                to, ignoredCarIds, checkForCars);
            if (fwd == null) return bwd;
            if (bwd == null) return fwd;
            return fwd.Value.Distance <= bwd.Value.Distance ? fwd : bwd;
        }

        public string? FindSharedNode(string segmentIdA, string segmentIdB)
        {
            if (!_segments.TryGetValue(segmentIdA, out var segA)) return null;
            if (!_segments.TryGetValue(segmentIdB, out var segB)) return null;

            if (segA.NodeA != null && (segA.NodeA == segB.NodeA || segA.NodeA == segB.NodeB))
                return segA.NodeA;
            if (segA.NodeB != null && (segA.NodeB == segB.NodeA || segA.NodeB == segB.NodeB))
                return segA.NodeB;
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
                    var (enter, exitNormal, exitReverse) = GetSwitchLegs(nodeId);
                    var candidates = new[] { enter, exitNormal, exitReverse }
                        .Where(s => s != null && s != currentSegId);
                    foreach (var nextSegId in candidates)
                    {
                        if (nextSegId == targetSegId) return true;
                    }
                    return false;
                }
                else
                {
                    var nextSegId = GetReachableSegment(currentSegId, currentEndA);
                    if (nextSegId == null) return false;
                    if (nextSegId == targetSegId) return true;

                    var nextNodeA = GetNodeAtEnd(nextSegId, true);
                    currentEndA = nextNodeA != nodeId;
                    currentSegId = nextSegId;
                }
            }
            return false;
        }

        // --- BFS routing ---

        private struct BfsState
        {
            public string SegmentId;
            public string EntryNode; // node through which we entered this segment
            public float Distance;
            public List<string> Path; // segment IDs in order
            public List<string> SwitchesVisited; // switch node IDs encountered along the route
        }

        private RouteResult? BfsRoute(GraphPosition from, GraphPosition to,
            bool checkForCars, IReadOnlyCollection<string>? ignoredCarIds)
        {
            // Same segment: return direct distance
            if (from.SegmentId == to.SegmentId)
            {
                var dist = Math.Abs(to.DistanceFromA - from.DistanceFromA);
                return new RouteResult(dist, 0, false, new List<string> { from.SegmentId });
            }

            if (!_segments.ContainsKey(from.SegmentId) || !_segments.ContainsKey(to.SegmentId))
                return null;

            // Determine the exit node based on facing direction
            // TowardEndA → exit through NodeA; TowardEndB → exit through NodeB
            bool exitEndA = from.Facing == Direction.TowardEndA;
            string exitNode = GetNodeAtEnd(from.SegmentId, exitEndA);
            if (exitNode == null)
                return null;

            // Distance from the start position to the exit node
            float distToExit = exitEndA ? from.DistanceFromA : (GetLength(from.SegmentId) - from.DistanceFromA);

            var queue = new Queue<BfsState>();
            var visited = new HashSet<string>(); // visited segment IDs

            // Seed the BFS with all neighbors reachable from the exit node
            var startNeighbors = GetNeighbors(from.SegmentId, exitNode);
            foreach (var neighborId in startNeighbors)
            {
                if (IsSegmentBlocked(neighborId, checkForCars, ignoredCarIds))
                    continue;

                var switchList = new List<string>();
                if (IsSwitch(exitNode))
                    switchList.Add(exitNode);

                var entryNode = exitNode; // we enter the neighbor through the shared exit node
                var distInNeighbor = GetDistanceThroughSegment(neighborId, entryNode);

                queue.Enqueue(new BfsState
                {
                    SegmentId = neighborId,
                    EntryNode = entryNode,
                    Distance = distToExit + distInNeighbor,
                    Path = new List<string> { from.SegmentId, neighborId },
                    SwitchesVisited = switchList
                });
            }

            visited.Add(from.SegmentId);

            while (queue.Count > 0)
            {
                var state = queue.Dequeue();

                // Check if we reached the destination segment
                if (state.SegmentId == to.SegmentId)
                {
                    // Adjust distance: we used full segment length, but we only need
                    // partial distance to the target position
                    float fullSegLen = GetLength(to.SegmentId);
                    float distFromEntry = GetDistanceFromEntry(to.SegmentId, state.EntryNode, to.DistanceFromA);
                    float distThroughSeg = GetDistanceThroughSegment(to.SegmentId, state.EntryNode);
                    float adjustedDist = state.Distance - distThroughSeg + distFromEntry;

                    int reversals = CountReversalsFromSwitchList(state.SwitchesVisited);

                    return new RouteResult(adjustedDist, reversals, false, state.Path);
                }

                if (visited.Contains(state.SegmentId))
                    continue;
                visited.Add(state.SegmentId);

                // Exit through the other end of this segment
                string otherNode = GetOtherNode(state.SegmentId, state.EntryNode);
                if (otherNode == null)
                    continue;

                var neighbors = GetNeighbors(state.SegmentId, otherNode);
                foreach (var neighborId in neighbors)
                {
                    if (visited.Contains(neighborId))
                        continue;
                    if (IsSegmentBlocked(neighborId, checkForCars, ignoredCarIds))
                        continue;

                    var switchList = new List<string>(state.SwitchesVisited);
                    if (IsSwitch(otherNode))
                        switchList.Add(otherNode);

                    var distInNeighbor = GetDistanceThroughSegment(neighborId, otherNode);

                    queue.Enqueue(new BfsState
                    {
                        SegmentId = neighborId,
                        EntryNode = otherNode,
                        Distance = state.Distance + distInNeighbor,
                        Path = new List<string>(state.Path) { neighborId },
                        SwitchesVisited = switchList
                    });
                }
            }

            return null; // no route found
        }

        /// <summary>
        /// Get all neighboring segments connected to segId through the given node.
        /// </summary>
        private List<string> GetNeighbors(string segId, string nodeId)
        {
            var result = new List<string>();
            if (IsSwitch(nodeId))
            {
                var (enter, exitN, exitR) = GetSwitchLegs(nodeId);
                if (enter != null && enter != segId) result.Add(enter);
                if (exitN != null && exitN != segId) result.Add(exitN);
                if (exitR != null && exitR != segId) result.Add(exitR);
            }
            else
            {
                // Non-switch node: find all other segments connected here
                foreach (var kv in _segments)
                {
                    if (kv.Key == segId) continue;
                    if (kv.Value.NodeA == nodeId || kv.Value.NodeB == nodeId)
                        result.Add(kv.Key);
                }
            }
            return result;
        }

        /// <summary>
        /// Check if a segment is blocked and should not be traversed.
        /// </summary>
        private bool IsSegmentBlocked(string segmentId, bool checkForCars, IReadOnlyCollection<string>? ignoredCarIds)
        {
            if (!checkForCars) return false;
            if (!_blockedCarSegments.Contains(segmentId)) return false;
            // In the mock, the segment ID doubles as the "car ID" for ignore purposes
            if (ignoredCarIds != null && ignoredCarIds.Contains(segmentId)) return false;
            return true;
        }

        /// <summary>
        /// Full length of a segment (used when traversing completely through it).
        /// </summary>
        private float GetDistanceThroughSegment(string segmentId, string entryNode)
        {
            return GetLength(segmentId);
        }

        /// <summary>
        /// Distance from the entry node to a specific position (DistanceFromA) within the segment.
        /// If entering through NodeA, distance is DistanceFromA.
        /// If entering through NodeB, distance is (Length - DistanceFromA).
        /// </summary>
        private float GetDistanceFromEntry(string segmentId, string entryNode, float distanceFromA)
        {
            if (!_segments.TryGetValue(segmentId, out var seg)) return 0f;
            if (seg.NodeA == entryNode)
                return distanceFromA;
            else
                return seg.Length - distanceFromA;
        }

        /// <summary>
        /// Count reversals from the list of switches visited along a route.
        /// A switch visited N times contributes N-1 reversals.
        /// </summary>
        private int CountReversalsFromSwitchList(List<string> switchesVisited)
        {
            if (switchesVisited.Count == 0) return 0;

            var counts = new Dictionary<string, int>();
            foreach (var sw in switchesVisited)
            {
                if (!counts.ContainsKey(sw))
                    counts[sw] = 0;
                counts[sw]++;
            }

            int reversals = 0;
            foreach (var kv in counts)
            {
                if (kv.Value > 1)
                    reversals += kv.Value - 1;
            }
            return reversals;
        }
    }
}
