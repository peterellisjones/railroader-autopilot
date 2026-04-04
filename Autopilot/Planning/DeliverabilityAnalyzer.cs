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
                List<(DirectedPosition loc, Car coupleTo, float availableSpace, int spanIndex)> destCandidates;
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

                // Pick the first candidate that passes route feasibility
                // AND has space for at least one car.
                //
                // For multi-segment spans, the far endpoint may show a
                // different reversal count than the near endpoint. But the
                // train enters the span from one side — if the approach
                // fails for the nearest candidate on a span, skip other
                // candidates on the SAME span. Different spans (independent
                // sidings) are checked independently.
                DirectedPosition destLocation = default;
                Car? coupleTarget = null;
                float availableSpace = 0f;
                int selectedSpanIndex = 0;
                bool foundDest = false;
                var failedSpans = new HashSet<int>();
                // Prefer candidates on a span the train is already on.
                var consistSegIds = new HashSet<string>();
                if (loco.LocationF.segment != null) consistSegIds.Add(loco.LocationF.segment.id);
                if (loco.LocationR.segment != null) consistSegIds.Add(loco.LocationR.segment.id);
                foreach (var c in group.Cars)
                {
                    if (c.EndA.Segment != null) consistSegIds.Add(c.EndA.Segment.id);
                    if (c.EndB.Segment != null) consistSegIds.Add(c.EndB.Segment.id);
                }

                // Try candidates on the train's span first, then the rest.
                var orderedCandidates = new System.Collections.Generic.List<(DirectedPosition loc, Car coupleTo, float availableSpace, int spanIndex)>();
                foreach (var c in destCandidates)
                    if (c.loc.Segment != null && consistSegIds.Contains(c.loc.Segment.id))
                        orderedCandidates.Add(c);
                foreach (var c in destCandidates)
                    if (c.loc.Segment == null || !consistSegIds.Contains(c.loc.Segment.id))
                        orderedCandidates.Add(c);

                foreach (var candidate in orderedCandidates)
                {
                    if (candidate.loc.Segment == null) continue;
                    if (candidate.availableSpace < 2f)
                        continue; // not enough space (need at least 2m overlap)
                    if (failedSpans.Contains(candidate.spanIndex))
                        continue; // approach already failed for this span

                    if (checker.CanDeliver(loco, group, candidate.loc))
                    {
                        destLocation = candidate.loc;
                        coupleTarget = candidate.coupleTo;
                        availableSpace = candidate.availableSpace;
                        selectedSpanIndex = candidate.spanIndex;
                        foundDest = true;
                        break;
                    }
                    else
                    {
                        failedSpans.Add(candidate.spanIndex);
                    }
                }

                if (!foundDest)
                    break; // no span is deliverable or has space — stop

                // Group consecutive cars going to the same physical track.
                // Each car uses its full length of space, but the last car
                // only needs 2m overlap with the span to count as delivered.
                const float MinOverlap = 2f;
                var firstGameCar = (car as CarAdapter)?.Car;
                var carGroup = new List<Car> { firstGameCar };
                float usedSpace = 0f; // first car only needs MinOverlap
                while (i + carGroup.Count < group.Cars.Count)
                {
                    var nextCar = group.Cars[i + carGroup.Count];
                    if (nextCar.Waybill == null)
                        break;
                    var nextDest = nextCar.Waybill.Value.Destination;
                    // Check same destination by identity, not by which segment
                    // GetDestinationLocation returns — that returns the global
                    // best candidate which may be on a different span/segment
                    // than the one selected for this delivery.
                    if (nextDest.Identifier != destination.Identifier)
                        break;
                    // Previous car now needs its full length plus coupling gap
                    // (it's no longer the last). Next car only needs MinOverlap.
                    var prevCar = carGroup[carGroup.Count - 1];
                    usedSpace += (prevCar?.carLength ?? car.CarLength) + AutopilotConstants.ConsistGapPerCar;
                    if (usedSpace + MinOverlap > availableSpace)
                        break;
                    carGroup.Add((nextCar as CarAdapter)?.Car);
                }

                steps.Add(new DeliveryStep(carGroup, destination, destLocation, coupleTarget, selectedSpanIndex));
                if (steps.Count >= maxSteps)
                    return steps;
                i += carGroup.Count;
            }

            return steps;
        }
    }
}
