using Model;
using Track;
using Autopilot.Model;

namespace Autopilot.Services
{
    public static class CoupleLocationCalculator
    {
        /// <summary>
        /// Compute the coupling waypoint location for the far end of an ICar target
        /// (the end facing away from the loco). Routes around the target and stops
        /// AutopilotConstants.CouplingOffsetDistance past its body.
        /// </summary>
        public static DirectedPosition GetCoupleLocation(ICar target, DirectedPosition locoPosition, Graph graph)
        {
            var nearestEnd = target.ClosestLogicalEndTo(locoPosition);
            var coupleEnd = nearestEnd == Car.LogicalEnd.A ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            return GetCoupleLocationForEnd(target, coupleEnd, graph);
        }

        /// <summary>
        /// Compute the coupling waypoint location for the far end of a Car target
        /// (the end facing away from the loco). Routes around the target and stops
        /// AutopilotConstants.CouplingOffsetDistance past its body.
        /// </summary>
        public static DirectedPosition GetCoupleLocation(Car target, DirectedPosition locoPosition, Graph graph)
        {
            // Convert to Location for game API, then back
            var loc = locoPosition.ToLocation();
            var nearestEnd = target.ClosestLogicalEndTo(loc, graph);
            var coupleEnd = nearestEnd == Car.LogicalEnd.A ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            return GetCoupleLocationForEnd(new CarAdapter(target), coupleEnd, graph);
        }

        /// <summary>
        /// Compute the coupling waypoint location for a specific logical end of an ICar target.
        /// Stops AutopilotConstants.CouplingOffsetDistance past the car body on that end.
        /// </summary>
        public static DirectedPosition GetCoupleLocationForEnd(ICar target, Car.LogicalEnd logicalEnd, Graph graph)
        {
            var carEnd = target.LogicalToEnd(logicalEnd);
            var endPos = carEnd == Car.End.F ? target.Front : target.Rear;
            // Move CouplingOffsetDistance past the car end
            var endLoc = endPos.ToLocation();
            var offsetLoc = graph.LocationByMoving(endLoc, AutopilotConstants.CouplingOffsetDistance, false, true);
            return DirectedPosition.FromLocation(offsetLoc);
        }
    }
}
