using System.Collections.Generic;
using System.Linq;
using Model;
using Model.Ops;
using RollingStock;
using Track;
using Track.Search;
using UnityEngine;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class RefuelPlanner
    {
        private readonly TrainService _trainService;

        // Cached per plan cycle — cleared by ClearCache()
        private List<FacilityInfo>? _cachedFacilities;

        public RefuelPlanner(TrainService trainService)
        {
            _trainService = trainService;
        }

        public void ClearCache()
        {
            _cachedFacilities = null;
        }

        /// <summary>
        /// Returns fuel types that are below the given threshold percentage.
        /// </summary>
        public List<string> GetLowFuelTypes(BaseLocomotive loco, int thresholdPercent)
        {
            if (thresholdPercent <= 0)
                return new List<string>();

            var fuelTypes = TrainService.GetFuelTypes(loco);
            var low = new List<string>();
            foreach (var ft in fuelTypes)
            {
                float level = _trainService.GetFuelLevel(loco, ft);
                if (level < thresholdPercent)
                    low.Add(ft);
            }
            return low;
        }

        /// <summary>
        /// Discover all fuel facilities in the world, filtered to the given fuel types.
        /// All fuel-type loaders are cached on first call (not just the requested types),
        /// so subsequent calls with different fuel types don't re-scan.
        /// </summary>
        public List<FacilityInfo> DiscoverFacilities(IEnumerable<string> fuelTypes)
        {
            var fuelTypeSet = new HashSet<string>(fuelTypes);

            if (_cachedFacilities != null)
                return _cachedFacilities.Where(f => fuelTypeSet.Contains(f.FuelType)).ToList();

            _cachedFacilities = new List<FacilityInfo>();
            var allFuelTypes = new HashSet<string> { "water", "coal", "diesel-fuel" };

            var loaders = Object.FindObjectsOfType<CarLoadTargetLoader>();
            foreach (var loader in loaders)
            {
                if (loader.load == null) continue;
                var loadId = loader.load.id;
                if (loadId == null || !allFuelTypes.Contains(loadId)) continue;

                if (!Graph.Shared.TryGetLocationFromWorldPoint(
                        loader.transform.position,
                        AutopilotConstants.LoaderLocationTolerance,
                        out Location loc))
                    continue;

                string? industryId = loader.sourceIndustry?.identifier;

                _cachedFacilities.Add(new FacilityInfo(
                    loadId,
                    DirectedPosition.FromLocation(loc),
                    loader.keyValueObject.RegisteredId,
                    industryId));
            }

            return _cachedFacilities.Where(f => fuelTypeSet.Contains(f.FuelType)).ToList();
        }

        /// <summary>
        /// Find the best facility to refuel at, considering coal-proximity scoring for steam locos.
        /// Returns null if no reachable facility found.
        /// </summary>
        public FacilityInfo? FindBestFacility(BaseLocomotive loco, List<string> lowFuelTypes)
        {
            if (lowFuelTypes.Count == 0)
                return null;

            // Determine all fuel types we might need (for facility discovery)
            var allFuelTypes = TrainService.GetFuelTypes(loco);
            var allFacilities = DiscoverFacilities(allFuelTypes);

            // Compute distances for all facilities
            foreach (var f in allFacilities)
            {
                var route = _trainService.GraphDistanceToLoco(loco, f.Location);
                f.Distance = route?.Distance ?? float.MaxValue;
            }

            var reachable = allFacilities
                .Where(f => f.Distance < float.MaxValue && !IsFacilityDepleted(f))
                .ToList();
            if (reachable.Count == 0)
                return null;

            // Diesel or single fuel type: just pick nearest matching
            if (allFuelTypes.Count == 1 || lowFuelTypes.Count == 1)
            {
                var targetType = GetPriorityFuelType(lowFuelTypes);
                return reachable
                    .Where(f => f.FuelType == targetType)
                    .OrderBy(f => f.Distance)
                    .FirstOrDefault();
            }

            // Steam with multiple low fuel types — apply coal-proximity scoring
            return FindBestSteamFacility(loco, reachable, lowFuelTypes);
        }

        /// <summary>
        /// Find best water facility for steam, considering coal proximity when coal is below 90%.
        /// Water first, then coal.
        /// </summary>
        private FacilityInfo? FindBestSteamFacility(BaseLocomotive loco,
            List<FacilityInfo> reachable, List<string> lowFuelTypes)
        {
            var waterFacilities = reachable.Where(f => f.FuelType == "water").ToList();
            var coalFacilities = reachable.Where(f => f.FuelType == "coal").ToList();

            // If water is low, find best water facility
            if (lowFuelTypes.Contains("water"))
            {
                if (waterFacilities.Count == 0)
                {
                    // No water — try coal if that's low
                    if (lowFuelTypes.Contains("coal"))
                        return coalFacilities.OrderBy(f => f.Distance).FirstOrDefault();
                    return null;
                }

                float coalLevel = _trainService.GetFuelLevel(loco, "coal");
                bool coalNeedsRefuel = coalLevel < AutopilotConstants.OpportunisticMaxPercent;

                if (coalNeedsRefuel && coalFacilities.Count > 0)
                {
                    // Score water facilities by combined distance: train->water + water->nearest coal
                    return ScoreWithCoalProximity(waterFacilities, coalFacilities);
                }

                // Coal is fine — just pick nearest water
                return waterFacilities.OrderBy(f => f.Distance).FirstOrDefault();
            }

            // Only coal is low
            if (lowFuelTypes.Contains("coal"))
                return coalFacilities.OrderBy(f => f.Distance).FirstOrDefault();

            return null;
        }

        /// <summary>
        /// Score water facilities by combined distance: train->water + water->nearest coal.
        /// Returns the water facility with lowest combined score.
        /// </summary>
        private FacilityInfo? ScoreWithCoalProximity(
            List<FacilityInfo> waterFacilities, List<FacilityInfo> coalFacilities)
        {
            FacilityInfo? best = null;
            float bestScore = float.MaxValue;

            foreach (var water in waterFacilities)
            {
                // Distance from water facility to nearest coal facility
                float nearestCoalDist = float.MaxValue;
                foreach (var coal in coalFacilities)
                {
                    var waterLoc = water.Location.ToLocation();
                    var coalLoc = coal.Location.ToLocation();
                    if (Graph.Shared.TryFindDistance(waterLoc, coalLoc, out float dist, out _))
                    {
                        if (dist < nearestCoalDist)
                            nearestCoalDist = dist;
                    }
                }

                float score = water.Distance + nearestCoalDist;
                if (score < bestScore)
                {
                    bestScore = score;
                    best = water;
                }
            }

            return best;
        }

        /// <summary>
        /// After completing a refuel, check if another fuel type's facility is nearby
        /// and that type is below 90%. Returns null if no opportunistic refuel needed.
        /// </summary>
        public FacilityInfo? FindNearbyOpportunistic(BaseLocomotive loco, string justRefueled)
        {
            var allFuelTypes = TrainService.GetFuelTypes(loco);
            var otherTypes = allFuelTypes.Where(ft => ft != justRefueled).ToList();
            if (otherTypes.Count == 0) return null;

            foreach (var ft in otherTypes)
            {
                float level = _trainService.GetFuelLevel(loco, ft);
                if (level >= AutopilotConstants.OpportunisticMaxPercent)
                    continue;

                var facilities = DiscoverFacilities(new[] { ft });
                foreach (var f in facilities)
                {
                    var route = _trainService.GraphDistanceToLoco(loco, f.Location);
                    if (route != null && route.Value.Distance <= AutopilotConstants.NearbyFacilityDistanceMeters
                        && !IsFacilityDepleted(f))
                    {
                        f.Distance = route.Value.Distance;
                        return f;
                    }
                }
            }

            return null;
        }

        /// <summary>Water before coal, coal before diesel.</summary>
        private static string GetPriorityFuelType(List<string> types)
        {
            if (types.Contains("water")) return "water";
            if (types.Contains("coal")) return "coal";
            return types[0];
        }

        /// <summary>
        /// Check if a facility's industry storage is empty (no fuel to dispense).
        /// Water towers (null IndustryId) are considered infinite — always returns false.
        /// </summary>
        private bool IsFacilityDepleted(FacilityInfo facility)
        {
            if (facility.IndustryId == null)
                return false; // Water towers are infinite

            var industry = OpsController.Shared?.IndustryForId(facility.IndustryId);
            if (industry == null)
                return false;

            var matchingLoad = industry.Storage.Loads()
                .FirstOrDefault(l => l.id == facility.FuelType);

            if (matchingLoad == null)
                return true;

            return industry.Storage.QuantityInStorage(matchingLoad) <= 0f;
        }
    }
}
