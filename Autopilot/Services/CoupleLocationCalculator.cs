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
            // The AE needs the waypoint slightly past the car body to
            // trigger coupling. Clamp to end of track if needed.
            var endLoc = endPos.ToLocation();
            Location offsetLoc;
            if (carEnd == Car.End.F)
                offsetLoc = graph.LocationByMoving(endLoc, AutopilotConstants.CouplingOffsetDistance, false, true);
            else
                offsetLoc = graph.LocationByMoving(endLoc, -AutopilotConstants.CouplingOffsetDistance, false, true).Flipped();
            return DirectedPosition.FromLocation(offsetLoc);
        }
    }
}
