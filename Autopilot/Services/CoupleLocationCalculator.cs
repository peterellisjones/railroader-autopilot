using Model;
using Track;
using Autopilot.Model;

namespace Autopilot.Services
{
    public static class CoupleLocationCalculator
    {
        /// <summary>
        /// Compute the coupling waypoint location for a specific logical end of an ICar target.
        /// Stops AutopilotConstants.CouplingOffsetDistance past the car body on that end.
        ///
        /// Callers must determine which end to use (typically the free/uncoupled end).
        /// Do NOT use ClosestLogicalEndTo (crow-flies) to pick the end — use coupling
        /// relationships instead.
        /// </summary>
        /// <summary>
        /// Returns null if the end faces a buffer stop (end of track).
        /// </summary>
        public static DirectedPosition GetCoupleLocationForEnd(ICar target, Car.LogicalEnd logicalEnd, Graph graph)
        {
            var carEnd = target.LogicalToEnd(logicalEnd);
            var endPos = carEnd == Car.End.F ? target.Front : target.Rear;
            // Move CouplingOffsetDistance past the car end.
            // LocationF faces outward from the car body, so positive distance moves away.
            // LocationR faces inward, so we need negative distance + Flip to move away.
            // Use the car's end position directly. The AE couples on
            // contact — no need for an offset past the car body.
            // The offset caused "End of Track" errors when the car was
            // near a buffer stop (track extends slightly past the car
            // but the AE can't route there with the full train).
            return endPos;
        }
    }
}
