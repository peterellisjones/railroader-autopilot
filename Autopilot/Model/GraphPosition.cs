using Autopilot.TrackGraph;

namespace Autopilot.Model
{
    /// <summary>
    /// A point on a track segment plus a facing direction — game-type-free.
    /// Uses string segment IDs (matching IGraphAdapter convention).
    /// Replaces DirectedPosition in interfaces and planning code.
    /// </summary>
    public readonly record struct GraphPosition(string SegmentId, float DistanceFromA, Direction Facing)
    {
        public GraphPosition Flipped => this with { Facing = Facing.Opposite() };

        public float DistanceFromB(IGraphAdapter graph) =>
            graph.GetLength(SegmentId) - DistanceFromA;

        public UndirectedGraphPosition Undirected => new(SegmentId, DistanceFromA);
    }

    /// <summary>
    /// A point on a track segment with no direction — game-type-free.
    /// Replaces TrackPosition in interfaces.
    /// </summary>
    public readonly record struct UndirectedGraphPosition(string SegmentId, float DistanceFromA);
}
