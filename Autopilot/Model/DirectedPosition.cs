using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A point on a track segment plus a facing direction.
    /// Primary replacement for game Location in domain code.
    /// Always stores DistanceFromA (canonical form).
    /// </summary>
    public readonly record struct DirectedPosition(TrackSegment Segment, float DistanceFromA, Direction Facing)
    {
        public TrackPosition Position => new(Segment, DistanceFromA);
        public float DistanceFromB => Segment.GetLength() - DistanceFromA;

        /// <summary>Distance to the segment end we're facing.</summary>
        public float DistanceAhead =>
            Facing == Direction.TowardEndB ? DistanceFromB : DistanceFromA;

        /// <summary>Distance to the segment end behind us.</summary>
        public float DistanceBehind =>
            Facing == Direction.TowardEndB ? DistanceFromA : DistanceFromB;

        /// <summary>Same position, opposite facing direction.</summary>
        public DirectedPosition Flipped => this with { Facing = Facing.Opposite() };

        /// <summary>
        /// Convert to game Location for API calls.
        /// Location.end = measurement origin = OPPOSITE of facing direction.
        /// </summary>
        public Location ToLocation()
        {
            // Facing TowardEndB => measurement origin is End.A (opposite)
            // Facing TowardEndA => measurement origin is End.B (opposite)
            var measureFrom = Facing == Direction.TowardEndB
                ? TrackSegment.End.A : TrackSegment.End.B;
            float distance = measureFrom == TrackSegment.End.A
                ? DistanceFromA : DistanceFromB;
            return new Location(Segment, distance, measureFrom);
        }

        /// <summary>
        /// Convert from game Location. THE critical translation point.
        /// Location.end is measurement origin; facing = opposite of that.
        /// </summary>
        public static DirectedPosition FromLocation(Location loc)
        {
            float distFromA = loc.EndIsA ? loc.distance : loc.segment.GetLength() - loc.distance;
            var facing = loc.EndIsA ? Direction.TowardEndB : Direction.TowardEndA;
            return new(loc.segment, distFromA, facing);
        }
    }
}
