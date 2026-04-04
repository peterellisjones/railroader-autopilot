using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A raw position on a destination span endpoint, used for approach direction
    /// and reversal analysis. Distinct from DirectedPosition to prevent confusion
    /// with coupling waypoints (which may be on a different segment due to offset).
    /// </summary>
    public readonly record struct SpanBoundary(TrackSegment Segment, float DistanceFromA, Direction Facing)
    {
        public DirectedPosition ToDirectedPosition() => new(Segment, DistanceFromA, Facing);

        public Location ToLocation() => ToDirectedPosition().ToLocation();
    }
}
