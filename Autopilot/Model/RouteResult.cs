namespace Autopilot.Model
{
    /// <summary>
    /// Result of a route search between two track positions.
    /// Not implicitly convertible to float — forces caller to consciously extract Distance.
    /// </summary>
    public readonly record struct RouteResult(
        float Distance,
        int ReversalCount,
        bool BlockedByCars,
        System.Collections.Generic.IReadOnlyList<string>? RouteSegmentIds = null
    )
    {
        public static bool operator <(RouteResult a, RouteResult b) => a.Distance < b.Distance;
        public static bool operator >(RouteResult a, RouteResult b) => a.Distance > b.Distance;
        public static bool operator <=(RouteResult a, RouteResult b) => a.Distance <= b.Distance;
        public static bool operator >=(RouteResult a, RouteResult b) => a.Distance >= b.Distance;
    }
}
