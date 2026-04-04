using System.Collections.Generic;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class FeasibilityChecker
    {
        private readonly TrainService _trainService;
        private readonly PlanningLogger _logger;
        public RouteChecker RouteChecker { get; }
        public ApproachAnalyzer ApproachAnalyzer { get; }
        public LoopValidator LoopValidator { get; }

        // Cached per plan cycle — GetLoopStatus is expensive (up to 5 loop searches)
        private LoopStatus? _cachedLoopStatus;
        private string? _cachedLoopStatusLocoId;

        public FeasibilityChecker(TrainService trainService)
        {
            _trainService = trainService;
            _logger = new PlanningLogger();
            RouteChecker = new RouteChecker(trainService, _logger);
            ApproachAnalyzer = new ApproachAnalyzer(RouteChecker, _logger);
            LoopValidator = new LoopValidator(RouteChecker, trainService, _logger);
        }

        public void ClearCache()
        {
            _cachedLoopStatus = null;
            _cachedLoopStatusLocoId = null;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Feasibility: {msg}");

        // --- Main API ---

        public bool CanDeliver(BaseLocomotive loco, CarGroup group, SpanBoundary destLocation)
        {
            if (group.IsEmpty) return false;

            if (!ApproachAnalyzer.CheckApproachDirection(loco, group, destLocation))
                return false;

            if (!CanRouteTo(loco, destLocation.ToDirectedPosition()))
                return false;

            return true;
        }

        public LoopStatus GetLoopStatus(BaseLocomotive loco)
        {
            if (_cachedLoopStatus != null && _cachedLoopStatusLocoId == loco.id)
                return _cachedLoopStatus;
            _cachedLoopStatus = LoopValidator.GetLoopStatus(loco);
            _cachedLoopStatusLocoId = loco.id;
            return _cachedLoopStatus;
        }

        public (DirectedPosition? location, string? loopKey) GetRepositionLocation(
            BaseLocomotive loco, IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys = null,
            List<SpanBoundary>? deliveryDestinations = null)
            => LoopValidator.GetRepositionLocation(loco, visitedSwitches, visitedLoopKeys, deliveryDestinations);

        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination)
            => RouteChecker.CanRouteTo(loco, destination);

        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
            => RouteChecker.CanRouteTo(loco, destination, trainLength, ignoredCars);

        public (bool found, List<RouteSearch.Step> steps) CanRouteToWithSteps(
            BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
            => RouteChecker.CanRouteToWithSteps(loco, destination, trainLength, ignoredCars);
    }
}
