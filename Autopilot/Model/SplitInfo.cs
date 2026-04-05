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
        public ICar SplitCar { get; }

        /// <summary>Which end of SplitCar faces the dropped cars.</summary>
        public Car.LogicalEnd SplitEnd { get; }

        /// <summary>Cars being dropped (need handbrakes, will be recoupled later).</summary>
        public IReadOnlyList<ICar> DroppedCars { get; }

        /// <summary>Location to return to for recoupling (first dropped car's position).</summary>
        public GraphPosition DropLocation { get; }

        /// <summary>First dropped car — the loco couples back to this.</summary>
        public ICar CoupleTarget { get; }

        /// <summary>Location for SetWaypointWithCouple (0.5m past couple target).</summary>
        public GraphCoupleWaypoint CoupleLocation { get; }

        public SplitInfo(ICar splitCar, Car.LogicalEnd splitEnd,
            IReadOnlyList<ICar> droppedCars, GraphPosition dropLocation,
            ICar coupleTarget, GraphCoupleWaypoint coupleLocation)
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
