namespace Autopilot
{
    public static class AutopilotConstants
    {
        /// <summary>Offset past car body for coupling waypoints (meters).</summary>
        public const float CouplingOffsetDistance = 0.5f;

        /// <summary>Gap between coupled cars in consist length calculations (meters).</summary>
        public const float ConsistGapPerCar = 1f;

        /// <summary>Timeout when train is stuck (not moving) during a movement action (seconds).</summary>
        public const float StuckTimeoutSeconds = 60f;

        /// <summary>Timeout waiting for decouple confirmation (seconds).</summary>
        public const float DecoupleWaitSeconds = 5f;
    }
}
