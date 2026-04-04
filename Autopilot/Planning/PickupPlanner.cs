using System.Collections.Generic;
using System.Linq;
using Model;
using Model.AI;
using Model.Ops;
using Track;
using UnityEngine;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class PickupPlanner
    {
        private readonly TrainService _trainService;

        public PickupPlanner(TrainService trainService)
        {
            _trainService = trainService;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Pickup: {msg}");

        /// <summary>
        /// Given a chain of cars ordered from approach end to buffer end,
        /// return consecutive target cars from the approach end.
        /// If the approach-end car is NOT a target, returns empty (blocked).
        /// </summary>
        public static List<ICar> FilterAccessibleTargets(
            List<ICar> chainFromApproach, HashSet<string> targetCarIds)
        {
            var targets = new List<ICar>();
            if (chainFromApproach.Count == 0)
                return targets;

            if (!targetCarIds.Contains(chainFromApproach[0].id))
                return targets;

            foreach (var car in chainFromApproach)
            {
                if (!targetCarIds.Contains(car.id))
                    break;
                targets.Add(car);
            }

            return targets;
        }

        /// <summary>
        /// Walk a car's coupled chain. Returns all cars ordered from
        /// the end with a free LogicalEnd.A to the end with a free LogicalEnd.B.
        /// </summary>
        public static List<ICar> GetCoupledChain(ICar startCar)
        {
            var current = startCar;
            while (current.CoupledTo(Car.LogicalEnd.A) != null)
                current = current.CoupledTo(Car.LogicalEnd.A)!;

            var chain = new List<ICar>();
            var visited = new HashSet<string>();
            while (current != null && !visited.Contains(current.id))
            {
                visited.Add(current.id);
                chain.Add(current);
                current = current.CoupledTo(Car.LogicalEnd.B);
            }

            return chain;
        }

        /// <summary>
        /// Check if a car matches the filter's base criteria: has an active waybill,
        /// switchlist membership (if applicable), and To:Destination match.
        /// Area/Industry checks that require game world data are handled separately
        /// by MatchesFromFilter and MatchesToFilter.
        /// </summary>
        public static bool MatchesFilter(ICar car, PickupFilter filter, HashSet<string> switchlistCarIds)
        {
            var waybill = car.Waybill;
            if (waybill == null) return false;
            if (waybill.Value.Completed) return false;

            if (filter.From.Mode == Autopilot.Model.FilterMode.Switchlist)
            {
                if (!switchlistCarIds.Contains(car.id)) return false;
            }

            if (filter.To.Mode == Autopilot.Model.FilterMode.Switchlist)
            {
                if (!switchlistCarIds.Contains(car.id)) return false;
            }

            if (filter.To.Mode == Autopilot.Model.FilterMode.Destination)
            {
                if (!filter.To.CheckedItems.Contains(waybill.Value.Destination.DisplayName))
                    return false;
            }

            return true;
        }

        /// <summary>
        /// Check if a car's physical location matches the From filter axis.
        /// Any/Switchlist modes are already handled by MatchesFilter.
        /// </summary>
        private static bool MatchesFromFilter(Car car, PickupFilter filter)
        {
            switch (filter.From.Mode)
            {
                case Autopilot.Model.FilterMode.Any:
                case Autopilot.Model.FilterMode.Switchlist:
                    return true;

                case Autopilot.Model.FilterMode.Area:
                {
                    var area = OpsController.Shared?.ClosestArea(car);
                    return area != null && filter.From.CheckedItems.Contains(area.name);
                }

                case Autopilot.Model.FilterMode.Industry:
                {
                    var ops = OpsController.Shared;
                    if (ops == null) return false;
                    var carPos = car.GetCenterPosition(Graph.Shared);
                    foreach (var area in ops.Areas)
                    {
                        if (!area.Contains(carPos)) continue;
                        foreach (var industry in area.Industries)
                        {
                            if (filter.From.CheckedItems.Contains(industry.name))
                                return true;
                        }
                    }
                    return false;
                }

                case Autopilot.Model.FilterMode.Destination:
                {
                    // From:Destination checks the car's waybill origin siding
                    var waybill = car.Waybill;
                    if (waybill == null) return false;
                    var origin = waybill.Value.Origin;
                    if (origin == null) return false;
                    return filter.From.CheckedItems.Contains(origin.Value.DisplayName);
                }

                default:
                    return true;
            }
        }

        /// <summary>
        /// Check if a car's waybill destination matches the To filter axis.
        /// Any/Switchlist/Destination modes are already handled by MatchesFilter.
        /// </summary>
        private static bool MatchesToFilter(Car car, PickupFilter filter)
        {
            switch (filter.To.Mode)
            {
                case Autopilot.Model.FilterMode.Any:
                case Autopilot.Model.FilterMode.Switchlist:
                case Autopilot.Model.FilterMode.Destination:
                    return true;

                case Autopilot.Model.FilterMode.Area:
                {
                    var waybill = car.Waybill;
                    if (waybill == null) return false;
                    var area = OpsController.Shared?.AreaForCarPosition(waybill.Value.Destination);
                    return area != null && filter.To.CheckedItems.Contains(area.name);
                }

                case Autopilot.Model.FilterMode.Industry:
                {
                    var waybill = car.Waybill;
                    if (waybill == null) return false;
                    var ops = OpsController.Shared;
                    if (ops == null) return false;
                    foreach (var area in ops.Areas)
                    {
                        foreach (var industry in area.Industries)
                        {
                            if (industry.Contains(waybill.Value.Destination)
                                && filter.To.CheckedItems.Contains(industry.name))
                                return true;
                        }
                    }
                    return false;
                }

                default:
                    return true;
            }
        }

        /// <summary>
        /// Find the nearest accessible car (or group) matching the filter.
        /// Returns null if no more targets are reachable.
        /// </summary>
        public PickupTarget? FindNextPickup(BaseLocomotive loco, PickupFilter filter,
            HashSet<Car>? skippedCars = null)
        {
            _trainService.ClearPlanCaches();
            var nearbyCars = _trainService.GetNearbyCars(loco);
            Log($"GetNearbyCars returned {nearbyCars.Count} cars");
            var graph = Graph.Shared;

            // Get switchlist car IDs if either axis uses switchlist mode
            HashSet<string> switchlistCarIds;
            if (filter.From.Mode == Autopilot.Model.FilterMode.Switchlist
                || filter.To.Mode == Autopilot.Model.FilterMode.Switchlist)
            {
                switchlistCarIds = _trainService.GetSwitchlistCarIds();
            }
            else
            {
                switchlistCarIds = new HashSet<string>();
            }

            // Get loco position for crow-flies distance pre-filter
            var locoPos = loco.GetCenterPosition(graph);

            // Find all uncoupled cars matching the filter
            var matchingCarIds = new HashSet<string>();
            var matchingCars = new Dictionary<string, Car>();
            foreach (var car in nearbyCars)
            {
                if (skippedCars != null && skippedCars.Contains(car)) continue;

                // Crow-flies distance pre-filter
                var carPos = car.GetCenterPosition(graph);
                if (filter.MaxDistance < float.MaxValue
                    && Vector3.Distance(locoPos, carPos) > filter.MaxDistance)
                    continue;

                // Base filter checks (waybill, switchlist, To:Destination)
                var wrapped = (ICar)new CarAdapter(car);
                if (!MatchesFilter(wrapped, filter, switchlistCarIds)) continue;

                // Game-world-dependent checks
                if (!MatchesFromFilter(car, filter)) continue;
                if (!MatchesToFilter(car, filter)) continue;

                matchingCarIds.Add(car.id);
                matchingCars[car.id] = car;
                Log($"Matching car: {car.DisplayName} on seg={car.LocationA.segment?.id}, visible={car.IsVisible}");
            }

            if (matchingCarIds.Count == 0)
            {
                Log($"No cars found matching filter");
                return null;
            }

            Log($"Found {matchingCarIds.Count} car(s) matching filter");

            var candidates = new List<(PickupTarget target, float distance)>();
            var processedChains = new HashSet<string>();

            foreach (var carId in matchingCarIds)
            {
                var car = matchingCars[carId];
                var wrapped = (ICar)new CarAdapter(car);
                var chain = GetCoupledChain(wrapped);

                var chainKey = chain[0].id;
                if (processedChains.Contains(chainKey))
                    continue;
                processedChains.Add(chainKey);

                // Determine approach end: which end of the chain is closer to the loco?
                var firstCar = chain[0];
                var lastCar = chain[chain.Count - 1];

                float distToFirst = _trainService.GraphDistanceToLoco(loco, firstCar.EndA)?.Distance ?? float.MaxValue;
                float distToLast = _trainService.GraphDistanceToLoco(loco, lastCar.EndB)?.Distance ?? float.MaxValue;

                // Try both orientations — the closer end might face a buffer stop
                var chainOrientations = new List<List<ICar>>();
                if (distToFirst <= distToLast)
                {
                    chainOrientations.Add(chain);
                    chainOrientations.Add(Enumerable.Reverse(chain).ToList());
                }
                else
                {
                    chainOrientations.Add(Enumerable.Reverse(chain).ToList());
                    chainOrientations.Add(chain);
                }

                // Try both orientations — each end of the chain is a
                // separate candidate.  When one end is blocked the other
                // may be reachable from a different approach direction.
                var coupled = _trainService.GetCoupled(loco);
                float trainLen = _trainService.GetTrainLength(loco);

                foreach (var orientation in chainOrientations)
                {
                    var targets = FilterAccessibleTargets(orientation, matchingCarIds);
                    if (targets.Count == 0)
                        continue;

                    var coupleTarget = (orientation[0] as CarAdapter)?.Car;
                    if (coupleTarget == null)
                        continue;

                    var target = orientation[0];

                    var endsToTry = new[] { Car.LogicalEnd.A, Car.LogicalEnd.B };
                    if (orientation.Count > 1)
                    {
                        var nextInChain = orientation[1];
                        var freeEnd = target.CoupledTo(Car.LogicalEnd.A)?.id == nextInChain.id
                            ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                        endsToTry = new[] { freeEnd };
                    }

                    CoupleWaypoint coupleLocation = default;
                    float distance = float.MaxValue;
                    bool reachable = false;

                    foreach (var logicalEnd in endsToTry)
                    {
                        var coupleWp = Services.CoupleLocationCalculator.GetCoupleLocationForEnd(target, logicalEnd, graph);
                        var testLoc = coupleWp.ToLocation();

                        var ignored = new System.Collections.Generic.HashSet<Car>(coupled);
                        float routeDist = float.MaxValue;
                        foreach (var locoLoc in new[] { loco.LocationF, loco.LocationR })
                        {
                            var impasse = new System.Collections.Generic.HashSet<Car>();
                            var routeSteps = new System.Collections.Generic.List<Track.Search.RouteSearch.Step>();
                            bool routeFound = Track.Search.RouteSearch.FindRoute(
                                graph, locoLoc, testLoc,
                                RouteChecker.DefaultHeuristic, routeSteps,
                                out Track.Search.RouteSearch.Metrics routeMetrics,
                                checkForCars: true, trainLength: trainLen,
                                trainMomentum: 0f, maxIterations: 5000,
                                checkForCarsIgnored: ignored,
                                checkForCarsImpasse: impasse);
                            if (routeFound && routeMetrics.Distance < routeDist)
                                routeDist = routeMetrics.Distance;
                        }

                        Log($"Trying {target.DisplayName} end {logicalEnd}: waypoint on {testLoc.segment?.id}|{testLoc.distance:F1}, dist={routeDist:F0}");
                        if (routeDist < distance)
                        {
                            coupleLocation = coupleWp;
                            distance = routeDist;
                            reachable = true;
                        }
                    }

                    if (!reachable)
                        continue;

                    var targetCars = targets.Select(c => (c as CarAdapter)?.Car).Where(c => c != null).ToList()!;
                    candidates.Add((new PickupTarget(
                        coupleTarget, coupleLocation, targetCars!), distance));
                }
            }

            if (candidates.Count == 0)
            {
                Log($"No accessible targets matching filter");
                return null;
            }

            candidates.Sort((a, b) => a.distance.CompareTo(b.distance));

            var best = candidates[0];
            Log($"Next pickup: {best.target.CoupleTarget.DisplayName} at distance {best.distance:F0}m " +
                $"({best.target.TargetCars.Count} target car(s))");
            return best.target;
        }

        /// <summary>
        /// Returns all cars matching the full filter (used for car count display).
        /// </summary>
        public List<Car> GetEligibleCars(BaseLocomotive loco, PickupFilter filter)
        {
            _trainService.ClearPlanCaches();
            var nearbyCars = _trainService.GetNearbyCars(loco);
            var graph = Graph.Shared;
            var locoPos = loco.GetCenterPosition(graph);

            HashSet<string> switchlistCarIds;
            if (filter.From.Mode == Autopilot.Model.FilterMode.Switchlist || filter.To.Mode == Autopilot.Model.FilterMode.Switchlist)
                switchlistCarIds = _trainService.GetSwitchlistCarIds();
            else
                switchlistCarIds = new HashSet<string>();

            var result = new List<Car>();
            foreach (var car in nearbyCars)
            {
                if (filter.MaxDistance < float.MaxValue)
                {
                    var carPos = car.GetCenterPosition(graph);
                    if (Vector3.Distance(locoPos, carPos) > filter.MaxDistance)
                        continue;
                }

                var wrapped = (ICar)new CarAdapter(car);
                if (!MatchesFilter(wrapped, filter, switchlistCarIds))
                    continue;

                if (!MatchesFromFilter(car, filter))
                    continue;

                if (!MatchesToFilter(car, filter))
                    continue;

                result.Add(car);
            }
            return result;
        }

        /// <summary>
        /// Returns checklist options for the From axis, cross-filtered by current To settings.
        /// </summary>
        public List<string> GetFromOptions(BaseLocomotive loco, PickupFilter filter)
        {
            // Get cars that pass the To filter (and base + distance), ignoring From
            var toOnlyFilter = new PickupFilter(FilterAxis.Any, filter.To, filter.MaxDistance, false);
            var cars = GetEligibleCars(loco, toOnlyFilter);
            return ExtractFromGroupingKeys(cars, filter.From.Mode);
        }

        /// <summary>
        /// Returns checklist options for the To axis, cross-filtered by current From settings.
        /// </summary>
        public List<string> GetToOptions(BaseLocomotive loco, PickupFilter filter)
        {
            var fromOnlyFilter = new PickupFilter(filter.From, FilterAxis.Any, filter.MaxDistance, false);
            var cars = GetEligibleCars(loco, fromOnlyFilter);
            return ExtractToGroupingKeys(cars, filter.To.Mode);
        }

        /// <summary>
        /// Extracts the sorted set of From-axis grouping keys from a list of cars.
        /// </summary>
        private List<string> ExtractFromGroupingKeys(List<Car> cars, Autopilot.Model.FilterMode mode)
        {
            var keys = new HashSet<string>();
            var opsController = OpsController.Shared;

            foreach (var car in cars)
            {
                string? key = null;
                switch (mode)
                {
                    case Autopilot.Model.FilterMode.Area:
                        var area = opsController?.ClosestArea(car);
                        key = area?.name;
                        break;
                    case Autopilot.Model.FilterMode.Industry:
                        if (opsController != null)
                        {
                            var carPos = car.GetCenterPosition(Graph.Shared);
                            foreach (var a in opsController.Areas)
                            {
                                if (!a.Contains(carPos)) continue;
                                foreach (var ind in a.Industries)
                                {
                                    key = ind.name;
                                    break;
                                }
                                if (key != null) break;
                            }
                        }
                        break;
                    case Autopilot.Model.FilterMode.Destination:
                        var wb = car.Waybill;
                        key = wb?.Origin?.DisplayName;
                        break;
                }

                if (!string.IsNullOrEmpty(key))
                    keys.Add(key!);
            }

            var result = keys.ToList();
            result.Sort();
            return result;
        }

        /// <summary>
        /// Extracts the sorted set of To-axis grouping keys from a list of cars.
        /// </summary>
        private List<string> ExtractToGroupingKeys(List<Car> cars, Autopilot.Model.FilterMode mode)
        {
            var keys = new HashSet<string>();
            var opsController = OpsController.Shared;

            foreach (var car in cars)
            {
                var waybill = car.Waybill;
                if (waybill == null) continue;

                string? key = null;
                switch (mode)
                {
                    case Autopilot.Model.FilterMode.Area:
                        var area = opsController?.AreaForCarPosition(waybill.Value.Destination);
                        key = area?.name;
                        break;
                    case Autopilot.Model.FilterMode.Industry:
                        if (opsController != null)
                        {
                            foreach (var a in opsController.Areas)
                            {
                                foreach (var ind in a.Industries)
                                {
                                    if (ind.Contains(waybill.Value.Destination))
                                    {
                                        key = ind.name;
                                        break;
                                    }
                                }
                                if (key != null) break;
                            }
                        }
                        break;
                    case Autopilot.Model.FilterMode.Destination:
                        key = waybill.Value.Destination.DisplayName;
                        break;
                }

                if (!string.IsNullOrEmpty(key))
                    keys.Add(key!);
            }

            var result = keys.ToList();
            result.Sort();
            return result;
        }

    }
}
