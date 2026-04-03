// Autopilot/Planning/RunaroundBuilder.cs
using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using static Autopilot.Services.CoupleLocationCalculator;

namespace Autopilot.Planning
{
    public class RunaroundBuilder
    {
        private readonly DeliverabilityAnalyzer _deliverabilityAnalyzer;

        public RunaroundBuilder(DeliverabilityAnalyzer deliverabilityAnalyzer)
        {
            _deliverabilityAnalyzer = deliverabilityAnalyzer;
        }

        public RunaroundAction? BuildRunaroundAction(BaseLocomotive loco, CarGroup group, Graph graph,
            TrainService trainService, FeasibilityChecker checker, IEnumerable<Car>? skippedCars = null)
        {
            if (group.IsEmpty) return null;

            // Find the optimal split point. After runaround, the inner cars become
            // the new tail. We want those inner cars to be deliverable. Find how
            // many cars from the inner end are deliverable (via Reversed),
            // then split to keep those with the loco.
            var flipped = group.Reversed();
            var directSteps = _deliverabilityAnalyzer.GetDeliverableSteps(loco, group, checker, skippedCars);
            var flippedSteps = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flipped, checker, skippedCars);

            // Mark which cars are directly deliverable (from the tail end)
            var directCars = new HashSet<Car>();
            foreach (var step in directSteps)
                foreach (var car in step.Cars) directCars.Add(car);

            // Mark which cars are deliverable after flip (from the inner end)
            var flippedCars = new HashSet<Car>();
            foreach (var step in flippedSteps)
                foreach (var car in step.Cars) flippedCars.Add(car);

            int carsToKeep = RunaroundSplitCalculator.CalculateCarsToKeep(
                group.Cars.Count,
                i => group.Cars[i].IsLocoOrTender,
                i => { var gc = (group.Cars[i] as CarAdapter)?.Car; return gc != null && directCars.Contains(gc); },
                i => { var gc = (group.Cars[i] as CarAdapter)?.Car; return gc != null && flippedCars.Contains(gc); });

            // group.Cars is ordered tail-to-loco. Cars to keep are at the inner end
            int splitIdx = group.Cars.Count - 1;

            // Skip locos/tenders
            while (splitIdx > 0 && group.Cars[splitIdx].IsLocoOrTender)
                splitIdx--;

            // Skip the deliverable cars (they stay with the loco)
            int kept = 0;
            while (splitIdx >= 0 && kept < carsToKeep)
            {
                splitIdx--;
                kept++;
            }

            // If all cars are deliverable after flip, detach ALL from the loco.
            if (splitIdx < 0)
            {
                splitIdx = group.Cars.Count - 1;
                while (splitIdx > 0 && group.Cars[splitIdx].IsLocoOrTender)
                    splitIdx--;
            }

            if (splitIdx < 0)
                return null;

            var splitCar = group.Cars[splitIdx];
            var coupleTarget = group.TailCar;

            // Determine which end of splitCar faces toward the loco
            Car.LogicalEnd splitEnd;
            if (splitIdx < group.Cars.Count - 1)
            {
                var nextInward = group.Cars[splitIdx + 1];
                // Find which end of splitCar is coupled to nextInward
                var coupledA = splitCar.CoupledTo(Car.LogicalEnd.A);
                bool aFacesNext = coupledA != null && coupledA.id == nextInward.id;
                splitEnd = aFacesNext ? Car.LogicalEnd.A : Car.LogicalEnd.B;
            }
            else if (group.Cars.Count >= 2)
            {
                // splitCar is LocomotiveEndCar — find end facing loco (away from group)
                var secondToLast = group.Cars[group.Cars.Count - 2];
                var coupledA = group.LocomotiveEndCar!.CoupledTo(Car.LogicalEnd.A);
                bool aFacesSecondToLast = coupledA != null && coupledA.id == secondToLast.id;
                // The end NOT facing secondToLast faces the loco (outward end)
                splitEnd = aFacesSecondToLast ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            }
            else
            {
                // Single car — the coupled end faces the loco
                splitEnd = splitCar.IsCoupled(Car.LogicalEnd.A)
                    ? Car.LogicalEnd.A : Car.LogicalEnd.B;
            }

            // Couple location: the free (uncoupled) end of the couple target.
            // This is the end facing AWAY from the rest of the car group —
            // after the runaround, the loco approaches from this side.
            // Don't use ClosestLogicalEndTo (crow-flies) as the loco may be
            // near a siding that makes crow-flies unreliable.
            var freeEnd = coupleTarget!.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var coupleLocation = GetCoupleLocationForEnd(coupleTarget!, freeEnd, graph);
            if (coupleLocation == null)
            {
                // Free end faces buffer stop — try the other end
                var otherEnd = freeEnd == Car.LogicalEnd.A ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                coupleLocation = GetCoupleLocationForEnd(coupleTarget!, otherEnd, graph);
            }
            if (coupleLocation == null)
                return null; // both ends face buffer stops

            // Only include cars being detached (tail up to split point)
            var disconnectedCars = group.Cars.Take(splitIdx + 1).Select(c => (c as CarAdapter)?.Car).Where(c => c != null).ToList()!;

            return new RunaroundAction(
                (splitCar as CarAdapter)?.Car!, splitEnd,
                (coupleTarget as CarAdapter)?.Car!, coupleLocation.Value,
                disconnectedCars!);
        }
    }
}
