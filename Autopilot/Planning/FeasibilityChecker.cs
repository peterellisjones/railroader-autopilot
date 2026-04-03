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

        public FeasibilityChecker(TrainService trainService)
        {
            _trainService = trainService;
            _logger = new PlanningLogger();
            RouteChecker = new RouteChecker(trainService, _logger);
            ApproachAnalyzer = new ApproachAnalyzer(RouteChecker, _logger);
            LoopValidator = new LoopValidator(RouteChecker, trainService, _logger);
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Feasibility: {msg}");

        // --- Main API ---

        public bool CanDeliver(BaseLocomotive loco, CarGroup group, DirectedPosition destLocation)
        {
            if (group.IsEmpty) return false;

            if (!ApproachAnalyzer.CheckApproachDirection(loco, group, destLocation))
                return false;

            // Route check uses game API — convert to Location
            if (!CanRouteTo(loco, destLocation))
                return false;

            return true;
        }

        public bool CanRunaround(BaseLocomotive loco, RunaroundAction runaround)
        {
            if (!LoopValidator.IsOnClearLoop(loco))
            {
                Log("Runaround feasibility: not on a clear loop — not feasible");
                return false;
            }

            Log("Runaround feasibility: on a clear loop — feasible");
            return true;
        }

        public bool IsOnClearLoop(BaseLocomotive loco)
            => LoopValidator.IsOnClearLoop(loco);

        public (DirectedPosition? location, string? loopKey) GetRepositionLocation(
            BaseLocomotive loco, IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys = null,
            List<DirectedPosition>? deliveryDestinations = null)
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
