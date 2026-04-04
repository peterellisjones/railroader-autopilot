using System.Collections.Generic;
using Model;

namespace Autopilot.Model
{
    /// <summary>
    /// Info for a train split: which car to uncouple at, which cars are dropped,
    /// and where to return for recoupling.
    /// </summary>
    public class SplitInfo
    {
        /// <summary>Last kept car — uncouple after this car's tail end.</summary>
        public Car SplitCar { get; }

        /// <summary>Which end of SplitCar faces the dropped cars.</summary>
        public Car.LogicalEnd SplitEnd { get; }

        /// <summary>Cars being dropped (need handbrakes, will be recoupled later).</summary>
        public List<Car> DroppedCars { get; }

        /// <summary>Location to return to for recoupling (first dropped car's position).</summary>
        public DirectedPosition DropLocation { get; }

        /// <summary>First dropped car — the loco couples back to this.</summary>
        public Car CoupleTarget { get; }

        /// <summary>Location for SetWaypointWithCouple (0.5m past couple target).</summary>
        public CoupleWaypoint CoupleLocation { get; }

        public SplitInfo(Car splitCar, Car.LogicalEnd splitEnd,
            List<Car> droppedCars, DirectedPosition dropLocation,
            Car coupleTarget, CoupleWaypoint coupleLocation)
        {
            SplitCar = splitCar;
            SplitEnd = splitEnd;
            DroppedCars = droppedCars;
            DropLocation = dropLocation;
            CoupleTarget = coupleTarget;
            CoupleLocation = coupleLocation;
        }
    }
}
