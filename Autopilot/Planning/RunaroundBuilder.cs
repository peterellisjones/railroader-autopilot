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
            TrainService trainService, FeasibilityChecker checker, IEnumerable<string>? skippedCarIds = null)
        {
            if (group.IsEmpty) return null;

            var flipped = group.Reversed();
            var directSteps = _deliverabilityAnalyzer.GetDeliverableSteps(loco, group, checker, skippedCarIds);
            var flippedSteps = _deliverabilityAnalyzer.GetDeliverableSteps(loco, flipped, checker, skippedCarIds);

            // Mark which cars are directly deliverable (from the tail end)
            var directCarIds = new HashSet<string>();
            foreach (var step in directSteps)
                foreach (var car in step.Cars) directCarIds.Add(car.id);

            // Mark which cars are deliverable after flip (from the inner end)
            var flippedCarIds = new HashSet<string>();
            foreach (var step in flippedSteps)
                foreach (var car in step.Cars) flippedCarIds.Add(car.id);

            int carsToKeep = RunaroundSplitCalculator.CalculateCarsToKeep(
                group.Cars.Count,
                i => group.Cars[i].IsLocoOrTender,
                i => directCarIds.Contains(group.Cars[i].id),
                i => flippedCarIds.Contains(group.Cars[i].id));

            // group.Cars is ordered tail-to-loco. Cars to keep are at the inner end
            int splitIdx = group.Cars.Count - 1;

            // Skip locos/tenders
            int tendersSkipped = 0;
            while (splitIdx > 0 && group.Cars[splitIdx].IsLocoOrTender)
            {
                splitIdx--;
                tendersSkipped++;
            }

            int freightToKeep = carsToKeep - tendersSkipped;
            int kept = 0;
            while (splitIdx >= 0 && kept < freightToKeep)
            {
                splitIdx--;
                kept++;
            }

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
                var coupledA = splitCar.CoupledTo(Car.LogicalEnd.A);
                bool aFacesNext = coupledA != null && coupledA.id == nextInward.id;
                splitEnd = aFacesNext ? Car.LogicalEnd.A : Car.LogicalEnd.B;
            }
            else if (group.Cars.Count >= 2)
            {
                var secondToLast = group.Cars[group.Cars.Count - 2];
                var coupledA = group.LocomotiveEndCar!.CoupledTo(Car.LogicalEnd.A);
                bool aFacesSecondToLast = coupledA != null && coupledA.id == secondToLast.id;
                splitEnd = aFacesSecondToLast ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            }
            else
            {
                splitEnd = splitCar.IsCoupled(Car.LogicalEnd.A)
                    ? Car.LogicalEnd.A : Car.LogicalEnd.B;
            }

            // Couple location: the free (uncoupled) end of the couple target.
            var freeEnd = coupleTarget!.CoupledTo(Car.LogicalEnd.A) != null
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;
            var coupleLocationCw = GetCoupleLocationForEnd(coupleTarget!, freeEnd, graph);

            // Convert to GraphCoupleWaypoint
            var coupleLocation = new GraphCoupleWaypoint(
                coupleLocationCw.Segment?.id, coupleLocationCw.DistanceFromA, coupleLocationCw.Facing);

            // Only include cars being detached (tail up to split point)
            var disconnectedCars = group.Cars.Take(splitIdx + 1).ToList();

            return new RunaroundAction(
                splitCar, splitEnd,
                coupleTarget!, coupleLocation,
                disconnectedCars);
        }
    }
}
