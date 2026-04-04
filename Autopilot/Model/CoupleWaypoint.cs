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
}
