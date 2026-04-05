using System.Collections.Generic;
using Autopilot.Model;

namespace Autopilot.TrackGraph
{
    /// <summary>
    /// Abstraction over the track graph, using string IDs instead of game types.
    /// Enables testing LoopFinder with mock graphs.
    /// </summary>
    public interface IGraphAdapter
    {
        // Segment operations

        /// <summary>Get the node ID at the given end (A or B) of a segment.</summary>
        string GetNodeAtEnd(string segmentId, bool endA);

        /// <summary>Check if the given node is at end A of the segment.</summary>
        bool IsEndA(string segmentId, string nodeId);

        /// <summary>Get the node at the opposite end of the segment from the given node.</summary>
        string GetOtherNode(string segmentId, string nodeId);

        /// <summary>Get the length of a segment in meters.</summary>
        float GetLength(string segmentId);

        /// <summary>Convert a game segment object to its string ID (registers it internally).</summary>
        string GetSegmentId(object segment);

        // Node operations

        /// <summary>Check if a node is a switch (turnout).</summary>
        bool IsSwitch(string nodeId);

        /// <summary>Get the three legs of a switch: enter (stem), exitNormal, exitReverse.</summary>
        (string enter, string exitNormal, string exitReverse) GetSwitchLegs(string nodeId);

        /// <summary>Get the fouling distance at a switch node.</summary>
        float GetFoulingDistance(string nodeId);

        // Navigation (for non-switch nodes)

        /// <summary>
        /// Get the next reachable segment from the given end of a segment
        /// (crossing a non-switch node).
        /// </summary>
        string GetReachableSegment(string segmentId, bool endA);

        // Route search

        /// <summary>
        /// Find a route between two directed positions, returning distance, reversal count,
        /// and whether the route was blocked by cars.
        /// Returns null if no route exists.
        /// </summary>
        RouteResult? FindRoute(GraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true);

        /// <summary>
        /// Find the best (shortest) route from an undirected position to a directed destination,
        /// trying both facing directions from the start.
        /// Returns null if no route exists in either direction.
        /// </summary>
        RouteResult? FindBestRoute(UndirectedGraphPosition from, GraphPosition to,
            IReadOnlyCollection<string>? ignoredCarIds = null, bool checkForCars = true);

        // Node navigation

        /// <summary>
        /// Find the node shared between two adjacent segments, or null if not adjacent.
        /// </summary>
        string? FindSharedNode(string segmentIdA, string segmentIdB);

        /// <summary>
        /// Determine which end of fromSegment leads toward toSegment (walking up to maxHops).
        /// Returns true if End A leads toward toSegment, false if End B, null if not reachable.
        /// </summary>
        bool? DirectionTowardIsEndA(string fromSegmentId, string toSegmentId, int maxHops = 5);
    }
}
