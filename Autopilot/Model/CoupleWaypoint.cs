using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A position computed by CoupleLocationCalculator for coupling operations.
    /// Passed to TrainService.SetWaypointWithCouple. Distinct from DirectedPosition
    /// to prevent passing arbitrary positions to coupling APIs.
    /// </summary>
    public readonly record struct CoupleWaypoint(TrackSegment Segment, float DistanceFromA, Direction Facing)
    {
        public DirectedPosition ToDirectedPosition() => new(Segment, DistanceFromA, Facing);

        public Location ToLocation() => ToDirectedPosition().ToLocation();
    }

    /// <summary>
    /// Abstract version of CoupleWaypoint using string segment IDs.
    /// Used by model types in the planning layer. Converted to CoupleWaypoint
    /// at the execution boundary via PlanUnwrapper.
    /// </summary>
    public readonly record struct GraphCoupleWaypoint(string SegmentId, float DistanceFromA, Direction Facing)
    {
        public GraphPosition ToGraphPosition() => new(SegmentId, DistanceFromA, Facing);
    }
}
