using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A raw position on a destination span endpoint, used for approach direction
    /// and reversal analysis. Distinct from DirectedPosition to prevent confusion
    /// with coupling waypoints (which may be on a different segment due to offset).
    /// Uses string segment IDs for testability.
    /// </summary>
    public readonly record struct SpanBoundary(string SegmentId, float DistanceFromA, Direction Facing)
    {
        public GraphPosition ToGraphPosition() => new(SegmentId, DistanceFromA, Facing);

        /// <summary>
        /// Convert to game DirectedPosition for execution layer code.
        /// Requires a GameGraphAdapter to resolve segment IDs.
        /// </summary>
        public DirectedPosition ToDirectedPosition(Autopilot.TrackGraph.GameGraphAdapter adapter)
            => adapter.ToDirectedPosition(ToGraphPosition());

        /// <summary>
        /// Convert to game Location for execution layer code.
        /// Requires a GameGraphAdapter to resolve segment IDs.
        /// </summary>
        public Location ToLocation(Autopilot.TrackGraph.GameGraphAdapter adapter)
            => ToDirectedPosition(adapter).ToLocation();
    }
}
