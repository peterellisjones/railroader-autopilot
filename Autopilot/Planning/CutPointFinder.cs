// Autopilot/Planning/CutPointFinder.cs
using System.Collections.Generic;
using Model;
using Autopilot.Model;

namespace Autopilot.Planning
{
    public static class CutPointFinder
    {
        public static (Car car, Car.LogicalEnd end) FindCutPoint(CarGroup group, List<Car> carsToDrop)
        {
            if (group.IsEmpty || carsToDrop.Count == 0)
                return (null, Car.LogicalEnd.A);

            // Find the innermost (closest to loco) car in the drop list.
            // Non-drop cars between the tail and the innermost drop car (e.g.,
            // tenders attached to a loco for sale) get detached along with it.
            int innermostDropIdx = -1;
            for (int i = 0; i < group.Cars.Count; i++)
            {
                var gameCar = (group.Cars[i] as CarAdapter)?.Car;
                if (gameCar != null && carsToDrop.Contains(gameCar))
                    innermostDropIdx = i;
            }

            if (innermostDropIdx < 0)
                return (null, Car.LogicalEnd.A);

            int dropCount = innermostDropIdx + 1; // everything from tail to innermostDropIdx

            if (dropCount >= group.Cars.Count)
            {
                // All cars on this side are being dropped. The cut point is the
                // innermost car (closest to loco), at the end facing the loco.
                var innerCar = group.Cars[group.Cars.Count - 1];
                Car.LogicalEnd innerEnd;

                if (group.Cars.Count > 1)
                {
                    // The loco-facing end is the one NOT coupled to the previous car
                    innerEnd = innerCar.CoupledTo(Car.LogicalEnd.A)?.id == group.Cars[group.Cars.Count - 2].id
                        ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                }
                else
                {
                    // Single car — find which end is coupled (to the loco).
                    // The coupled end is the one to uncouple from.
                    innerEnd = innerCar.IsCoupled(Car.LogicalEnd.A)
                        ? Car.LogicalEnd.A : Car.LogicalEnd.B;
                }

                return ((innerCar as CarAdapter)?.Car, innerEnd);
            }

            var cutCar = group.Cars[dropCount - 1]; // last car being dropped
            var nextCar = group.Cars[dropCount]; // first car staying

            // Use actual CoupledTo to find the correct end
            var cutEnd = cutCar.CoupledTo(Car.LogicalEnd.A)?.id == nextCar.id
                ? Car.LogicalEnd.A : Car.LogicalEnd.B;

            return ((cutCar as CarAdapter)?.Car, cutEnd);
        }
    }
}
