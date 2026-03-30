using System.Collections.Generic;
using Model;
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

        public RouteResult? FindRoute(DirectedPosition from, DirectedPosition to,
            IReadOnlyCollection<Car>? ignoredCars = null, bool checkForCars = true)
        {
            return null;
        }

        public RouteResult? FindBestRoute(TrackPosition from, DirectedPosition to,
            IReadOnlyCollection<Car>? ignoredCars = null, bool checkForCars = true)
        {
            return null;
        }
    }
}
