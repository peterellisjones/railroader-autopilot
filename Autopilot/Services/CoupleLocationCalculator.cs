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
        public static DirectedPosition GetCoupleLocationForEnd(ICar target, Car.LogicalEnd logicalEnd, Graph graph)
        {
            var carEnd = target.LogicalToEnd(logicalEnd);
            var endPos = carEnd == Car.End.F ? target.Front : target.Rear;
            // Move CouplingOffsetDistance past the car end.
            // LocationF faces outward from the car body, so positive distance moves away.
            // LocationR faces inward, so we need negative distance + Flip to move away.
            var endLoc = endPos.ToLocation();
            try
            {
                Location offsetLoc;
                if (carEnd == Car.End.F)
                    offsetLoc = graph.LocationByMoving(endLoc, AutopilotConstants.CouplingOffsetDistance, false);
                else
                    offsetLoc = graph.LocationByMoving(endLoc, -AutopilotConstants.CouplingOffsetDistance, false).Flipped();
                return DirectedPosition.FromLocation(offsetLoc);
            }
            catch
            {
                // End of track — offset would go past the buffer stop.
                // Use the car end position directly; the AE couples on contact.
                return endPos;
            }
        }
    }
}
