namespace Autopilot
{
    public static class AutopilotConstants
    {
        /// <summary>Offset past car body for coupling waypoints (meters).</summary>
        public const float CouplingOffsetDistance = 0.5f;

        /// <summary>Gap between coupled cars in consist length calculations (meters).</summary>
        public const float ConsistGapPerCar = 1f;

        /// <summary>Timeout waiting for decouple confirmation (seconds).</summary>
        public const float DecoupleWaitSeconds = 5f;

        /// <summary>Speed limit while approaching a fuel facility (mph).</summary>
        public const int RefuelApproachSpeedMph = 5;

        /// <summary>Tank considered full when remaining capacity is below this (game units).</summary>
        public const float FullThresholdUnits = 25f;

        /// <summary>Graph distance (meters) for opportunistic nearby refueling.</summary>
        public const float NearbyFacilityDistanceMeters = 100f;

        /// <summary>Don't opportunistically refuel above this percentage.</summary>
        public const float OpportunisticMaxPercent = 90f;

        /// <summary>Radius for matching a CarLoadTargetLoader to a track location (game units).</summary>
        public const float LoaderLocationTolerance = 10f;
    }
}
