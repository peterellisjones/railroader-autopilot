using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// Direction of travel along a track segment. Unlike TrackSegment.End which
    /// identifies an endpoint, Direction indicates which way something faces or moves.
    /// </summary>
    public enum Direction
    {
        TowardEndA,
        TowardEndB
    }

    public static class DirectionExtensions
    {
        public static Direction Opposite(this Direction d) =>
            d == Direction.TowardEndA ? Direction.TowardEndB : Direction.TowardEndA;

        /// <summary>The segment End this direction faces toward.</summary>
        public static TrackSegment.End ToSegmentEnd(this Direction d) =>
            d == Direction.TowardEndA ? TrackSegment.End.A : TrackSegment.End.B;

        /// <summary>Create a Direction from a segment End (toward that end).</summary>
        public static Direction ToDirection(this TrackSegment.End end) =>
            end == TrackSegment.End.A ? Direction.TowardEndA : Direction.TowardEndB;
    }
}
