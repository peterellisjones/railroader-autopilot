using System.Collections.Generic;
using Model;

namespace Autopilot.Model
{
    public class RunaroundAction
    {
        /// <summary>Car at the split point (loco-side of the group to runaround).</summary>
        public Car SplitCar { get; }

        /// <summary>Which end of SplitCar to uncouple (the end facing the loco).</summary>
        public Car.LogicalEnd SplitEnd { get; }

        /// <summary>Car at the far end of the group — the loco will couple to this car.</summary>
        public Car CoupleTarget { get; }

        /// <summary>Location at the far end of CoupleTarget — forces AE to route around.</summary>
        public CoupleWaypoint CoupleLocation { get; }

        /// <summary>Cars being disconnected (need handbrakes).</summary>
        public List<Car> DisconnectedCars { get; }

        public RunaroundAction(Car splitCar, Car.LogicalEnd splitEnd, Car coupleTarget,
            CoupleWaypoint coupleLocation, List<Car> disconnectedCars)
        {
            SplitCar = splitCar;
            SplitEnd = splitEnd;
            CoupleTarget = coupleTarget;
            CoupleLocation = coupleLocation;
            DisconnectedCars = disconnectedCars;
        }
    }
}
