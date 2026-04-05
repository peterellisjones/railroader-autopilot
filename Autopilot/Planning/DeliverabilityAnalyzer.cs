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
        private readonly DestinationSelector? _destinationSelector;
        private readonly ITrainService? _iTrainService;

        public DeliverabilityAnalyzer(DestinationSelector destinationSelector)
        {
            _destinationSelector = destinationSelector;
        }

        /// <summary>Constructor for testable planning.</summary>
        public DeliverabilityAnalyzer(ITrainService trainService)
        {
            _iTrainService = trainService;
        }

        public List<DeliveryStep> GetDeliverableSteps(BaseLocomotive loco, CarGroup group,
            FeasibilityChecker checker, IEnumerable<string>? skippedCarIds = null, int maxSteps = int.MaxValue)
        {
            var steps = new List<DeliveryStep>();
            if (group.IsEmpty) return steps;

            int i = 0;
            while (i < group.Cars.Count)
            {
                var car = group.Cars[i];

                // A skipped car (siding overflow) blocks access to cars behind it.
                if (skippedCarIds != null && skippedCarIds.Contains(car.id))
                    break;

                var waybill = car.Waybill;

                if (waybill == null)
                {
                    i++;
                    continue;
                }

                var destination = waybill.Value.Destination;
                List<(DirectedPosition loc, Car coupleTo, float availableSpace, int spanIndex, SpanBoundary approachTarget)> destCandidates;
                try
                {
                    destCandidates = _destinationSelector!.GetDestinationCandidates(destination, loco);
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
                var orderedCandidates = new System.Collections.Generic.List<(DirectedPosition loc, Car coupleTo, float availableSpace, int spanIndex, SpanBoundary approachTarget)>();
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
                        continue;
                    if (failedSpans.Contains(candidate.spanIndex))
                        continue;

                    if (checker.CanDeliver(loco, group, candidate.approachTarget))
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
                    break;

                // Group consecutive cars going to the same physical track.
                var carGroup = new List<ICar> { car };
                float usedSpace = 0f;
                while (i + carGroup.Count < group.Cars.Count)
                {
                    var nextCar = group.Cars[i + carGroup.Count];
                    if (nextCar.Waybill == null)
                        break;
                    var nextDest = nextCar.Waybill.Value.Destination;
                    if (nextDest.Identifier != destination.Identifier)
                        break;
                    var prevCar = carGroup[carGroup.Count - 1];
                    usedSpace += prevCar.CarLength + AutopilotConstants.ConsistGapPerCar;
                    if (usedSpace + 2f > availableSpace)
                        break;
                    carGroup.Add(nextCar);
                }

                // Convert game types to abstract types for the DeliveryStep
                var destGp = new GraphPosition(destLocation.Segment?.id, destLocation.DistanceFromA, destLocation.Facing);
                ICar? coupleTargetICar = coupleTarget != null ? new CarAdapter(coupleTarget) : null;

                steps.Add(new DeliveryStep(carGroup, destination.Identifier, destination.DisplayName,
                    destGp, coupleTargetICar, selectedSpanIndex));
                if (steps.Count >= maxSteps)
                    return steps;
                i += carGroup.Count;
            }

            return steps;
        }

        // =================================================================
        // Testable overload using ITrainService (no game types)
        // =================================================================

        /// <summary>
        /// Get deliverable steps using ITrainService for destination queries.
        /// Uses FeasibilityChecker's abstract CanDeliver overload for feasibility checks.
        /// </summary>
        public List<DeliveryStep> GetDeliverableSteps(CarGroup group,
            FeasibilityChecker checker, IReadOnlyCollection<string>? skippedCarIds = null,
            int maxSteps = int.MaxValue)
        {
            var steps = new List<DeliveryStep>();
            if (group.IsEmpty || _iTrainService == null) return steps;

            int i = 0;
            while (i < group.Cars.Count)
            {
                var car = group.Cars[i];

                // A skipped car (siding overflow) blocks access to cars behind it.
                if (skippedCarIds != null && skippedCarIds.Contains(car.id))
                    break;

                if (!_iTrainService.HasWaybill(car))
                {
                    i++;
                    continue;
                }

                var destTrackId = _iTrainService.GetDestinationTrackId(car);
                var destName = _iTrainService.GetDestinationName(car);
                if (destTrackId == null || destName == null)
                {
                    i++;
                    continue;
                }

                var candidates = _iTrainService.GetDestinationCandidates(car);
                if (candidates.Count == 0)
                {
                    i++;
                    continue;
                }

                GraphPosition destLocation = default;
                ICar? coupleTarget = null;
                float availableSpace = 0f;
                int selectedSpanIndex = 0;
                bool foundDest = false;
                var failedSpans = new HashSet<int>();

                foreach (var candidate in candidates)
                {
                    if (candidate.Location.SegmentId == null) continue;
                    if (candidate.AvailableSpace < 2f) continue;
                    if (failedSpans.Contains(candidate.SpanIndex)) continue;

                    // Use the abstract CanDeliver that takes GraphPositions
                    if (group.TailOutwardEnd.HasValue && group.TailInwardEnd.HasValue
                        && checker.CanDeliver(group.TailOutwardEnd.Value,
                            group.TailInwardEnd.Value, candidate.ApproachTarget))
                    {
                        destLocation = candidate.Location;
                        coupleTarget = candidate.CoupleTarget;
                        availableSpace = candidate.AvailableSpace;
                        selectedSpanIndex = candidate.SpanIndex;
                        foundDest = true;
                        break;
                    }
                    else
                    {
                        failedSpans.Add(candidate.SpanIndex);
                    }
                }

                if (!foundDest)
                    break;

                // Group consecutive cars going to the same destination track.
                var carGroup = new List<ICar> { car };
                float usedSpace = 0f;
                while (i + carGroup.Count < group.Cars.Count)
                {
                    var nextCar = group.Cars[i + carGroup.Count];
                    if (!_iTrainService.HasWaybill(nextCar))
                        break;
                    var nextDestTrackId = _iTrainService.GetDestinationTrackId(nextCar);
                    if (nextDestTrackId != destTrackId)
                        break;
                    var prevCar = carGroup[carGroup.Count - 1];
                    usedSpace += prevCar.CarLength + AutopilotConstants.ConsistGapPerCar;
                    if (usedSpace + 2f > availableSpace)
                        break;
                    carGroup.Add(nextCar);
                }

                steps.Add(new DeliveryStep(carGroup, destTrackId, destName,
                    destLocation, coupleTarget, selectedSpanIndex));
                if (steps.Count >= maxSteps)
                    return steps;
                i += carGroup.Count;
            }

            return steps;
        }
    }
}
