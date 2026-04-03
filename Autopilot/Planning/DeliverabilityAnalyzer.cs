// Autopilot/Planning/DeliverabilityAnalyzer.cs
using System.Collections.Generic;
using System.Linq;
using Model;
using Model.Ops;
using Track;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class DeliverabilityAnalyzer
    {
        private readonly DestinationSelector _destinationSelector;

        public DeliverabilityAnalyzer(DestinationSelector destinationSelector)
        {
            _destinationSelector = destinationSelector;
        }

        public List<DeliveryStep> GetDeliverableSteps(BaseLocomotive loco, CarGroup group,
            FeasibilityChecker checker, IEnumerable<Car>? skippedCars = null, int maxSteps = int.MaxValue)
        {
            var steps = new List<DeliveryStep>();
            if (group.IsEmpty) return steps;

            int i = 0;
            while (i < group.Cars.Count)
            {
                var car = group.Cars[i];

                // A skipped car (siding overflow) blocks access to cars behind it.
                // Can't uncouple past it — must split or runaround first.
                var gameCarCheck = (car as CarAdapter)?.Car;
                if (skippedCars != null && gameCarCheck != null && skippedCars.Contains(gameCarCheck))
                    break;

                var waybill = car.Waybill;

                if (waybill == null)
                {
                    i++;
                    continue;
                }

                var destination = waybill.Value.Destination;
                List<(DirectedPosition loc, Car coupleTo, float availableSpace)> destCandidates;
                try
                {
                    destCandidates = _destinationSelector.GetDestinationCandidates(destination, loco);
                }
                catch
                {
                    i++;
                    continue;
                }

                if (destCandidates.Count == 0)
                {
                    i++;
                    continue;
                }

                // Pick the first candidate span that passes route feasibility
                // AND has space for at least one car.
                DirectedPosition destLocation = default;
                Car? coupleTarget = null;
                float availableSpace = 0f;
                bool foundDest = false;
                foreach (var candidate in destCandidates)
                {
                    if (candidate.loc.Segment == null) continue;
                    if (candidate.availableSpace < car.CarLength)
                        continue; // not enough space on this span
                    if (checker.CanDeliver(loco, group, candidate.loc))
                    {
                        destLocation = candidate.loc;
                        coupleTarget = candidate.coupleTo;
                        availableSpace = candidate.availableSpace;
                        foundDest = true;
                        break;
                    }
                }

                if (!foundDest)
                    break; // no span is deliverable or has space — stop

                // Group consecutive cars going to the same physical track,
                // limited by available space on the destination.
                var firstGameCar = (car as CarAdapter)?.Car;
                var carGroup = new List<Car> { firstGameCar };
                float usedSpace = firstGameCar?.carLength ?? car.CarLength;
                while (i + carGroup.Count < group.Cars.Count)
                {
                    var nextCar = group.Cars[i + carGroup.Count];
                    if (nextCar.Waybill == null)
                        break;
                    var nextDest = nextCar.Waybill.Value.Destination;
                    DirectedPosition nextDestLoc;
                    try
                    {
                        nextDestLoc = _destinationSelector.GetDestinationLocation(nextDest, loco);
                    }
                    catch { break; }
                    if (nextDestLoc.Segment == null || nextDestLoc.Segment != destLocation.Segment)
                        break;
                    float nextLen = (nextCar as CarAdapter)?.Car?.carLength ?? nextCar.CarLength;
                    if (usedSpace + nextLen + AutopilotConstants.ConsistGapPerCar > availableSpace)
                        break; // no room for this car
                    carGroup.Add((nextCar as CarAdapter)?.Car);
                    usedSpace += nextLen + AutopilotConstants.ConsistGapPerCar;
                }

                steps.Add(new DeliveryStep(carGroup, destination, destLocation, coupleTarget));
                if (steps.Count >= maxSteps)
                    return steps;
                i += carGroup.Count;
            }

            return steps;
        }
    }
}
