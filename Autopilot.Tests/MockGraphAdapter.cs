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
            // Stub — BFS implementation comes in Task 4
            return null;
        }

        public RouteResult? FindBestRoute(UndirectedGraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true)
        {
            // Stub — BFS implementation comes in Task 4
            return null;
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
    }
}
