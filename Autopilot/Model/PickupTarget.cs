// Autopilot/Model/PickupTarget.cs
using System.Collections.Generic;
using System.Linq;
using Model;

namespace Autopilot.Model
{
    /// <summary>
    /// Represents a single pickup operation: the car to couple with,
    /// the target cars that will be collected, and the destination name.
    /// Non-target cars coupled behind the targets come along for the ride
    /// (no uncoupling — they stay in the consist).
    /// </summary>
    public class PickupTarget
    {
        /// <summary>The outermost car — the one the loco couples to.</summary>
        public Car CoupleTarget { get; }

        /// <summary>Location for SetWaypointWithCouple.</summary>
        public CoupleWaypoint CoupleLocation { get; }

        /// <summary>Consecutive target cars from the approach end.</summary>
        public List<Car> TargetCars { get; }

        /// <summary>Destination name for display.</summary>
        public string DestinationName { get; }

        public PickupTarget(Car coupleTarget, CoupleWaypoint coupleLocation,
            List<Car> targetCars, string destinationName)
        {
            CoupleTarget = coupleTarget;
            CoupleLocation = coupleLocation;
            TargetCars = targetCars;
            DestinationName = destinationName;
        }

        public string CarNames => string.Join(", ", TargetCars.Select(c => c.DisplayName));
    }
}
