using System;
using System.Linq;
using Game.Messages;
using Game.State;
using KeyValue.Runtime;
using Model;
using Model.AI;
using Model.Ops;
using UI.EngineControls;
using Model.Ops.Definition;
using RollingStock;
using Track;
using Track.Search;
using UnityEngine;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class RefuelAction : IAction
    {
        private enum Phase
        {
            MovingToFacility,
            WaitingForStop,
            Positioning,
            Refueling,
            Cleanup
        }

        private readonly FacilityInfo _facility;
        private Phase _phase;
        private float _stuckTimer;
        private float _statusTimer;
        private float _refuelStartLevel = -1f;
        private float _noProgressTimer;
        private string _statusMessage;
        private string? _initError;

        // Speed state — restore original speed after refuel approach
        private int _previousMaxSpeed;

        // Loader state — resolved once when we start refueling
        private CarLoaderSequencer? _sequencer;
        private bool _loaderActivated;

        /// <summary>The fuel type this action is refueling.</summary>
        public string FuelType => _facility.FuelType;

        public string StatusMessage => _statusMessage;

        public RefuelAction(FacilityInfo facility, BaseLocomotive loco, TrainService trainService)
        {
            _facility = facility;
            _statusMessage = $"Moving to {facility.FuelType} facility...";
            _phase = Phase.MovingToFacility;

            try
            {
                // Save original max speed before overriding for slow approach
                var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
                _previousMaxSpeed = persistence.Orders.MaxSpeedMph;

                trainService.SetWaypoint(loco, facility.Location);

                // Override AE speed to approach speed for slow approach
                var helper = new AutoEngineerOrdersHelper(loco, persistence);
                helper.SetOrdersValue(null, null, AutopilotConstants.RefuelApproachSpeedMph, null, null);

                Loader.Mod.Logger.Log($"Autopilot RefuelAction: moving to {facility.FuelType} facility " +
                    $"at {facility.Location.Segment?.id}|{facility.Location.DistanceFromA:F1}, " +
                    $"distance={facility.Distance:F0}m, approach speed={AutopilotConstants.RefuelApproachSpeedMph}mph");
            }
            catch (Exception ex)
            {
                _initError = $"Failed to set waypoint for refuel: {ex.Message}";
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: {_initError}");
            }
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            if (_initError != null)
                return new ActionFailed(_initError);

            switch (_phase)
            {
                case Phase.MovingToFacility:
                    return TickMovingToFacility(loco, trainService);
                case Phase.WaitingForStop:
                    return TickWaitingForStop(loco, trainService);
                case Phase.Positioning:
                    return TickPositioning(loco, trainService);
                case Phase.Refueling:
                    return TickRefueling(loco, trainService);
                case Phase.Cleanup:
                    return TickCleanup(loco, trainService);
                default:
                    return new ActionFailed("RefuelAction: unknown phase.");
            }
        }

        private ActionOutcome TickMovingToFacility(BaseLocomotive loco, TrainService trainService)
        {
            if (!trainService.IsWaypointMode(loco))
                return new ActionFailed("Player took manual control — autopilot paused.");

            if (trainService.IsWaypointSatisfied(loco) && trainService.IsStopped(loco))
            {
                _phase = Phase.WaitingForStop;
                _statusMessage = $"Arriving at {_facility.FuelType} facility...";
                _statusTimer = 0f;
            }
            else if (trainService.IsStopped(loco))
            {
                _stuckTimer += AutopilotController.TickInterval;
                if (_stuckTimer > Loader.Settings.stuckTimeoutSeconds)
                    return new ActionFailed(
                        $"Train stuck for {Loader.Settings.stuckTimeoutSeconds:0}s moving to " +
                        $"{_facility.FuelType} facility. Is the route blocked?");
            }
            else
            {
                _stuckTimer = 0f;
            }

            return new InProgress();
        }

        private ActionOutcome TickWaitingForStop(BaseLocomotive loco, TrainService trainService)
        {
            if (!trainService.IsStoppedForDuration(loco, 1.0f))
                return new InProgress();

            // Check if fuel car needs a nudge to align with the loader
            var fuelCar = trainService.GetFuelCar(loco);
            var loaderLoc = _facility.Location.ToLocation();
            var graph = Graph.Shared;

            bool foundF = graph.TryFindDistance(fuelCar.LocationF, loaderLoc, out float distF, out _);
            bool foundR = graph.TryFindDistance(fuelCar.LocationR, loaderLoc, out float distR, out _);

            if (!foundF && !foundR)
            {
                // Can't determine distance — assume close enough, proceed to refuel
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: TryFindDistance failed for both ends " +
                    $"of fuel car {fuelCar.DisplayName}, assuming close enough");
                return TransitionToRefueling(loco, trainService);
            }

            // Compute distance from the fuel car's CENTER to the loader.
            // Using (distF + distR) / 2 centers the car on the loader,
            // rather than just bringing the nearest end close.
            float centerDist;
            if (foundF && foundR)
                centerDist = (distF + distR) / 2f;
            else if (foundF)
                centerDist = distF;
            else
                centerDist = distR;

            Loader.Mod.Logger.Log($"Autopilot RefuelAction: fuel car {fuelCar.DisplayName} " +
                $"distance to loader: F={distF:F1}m(found={foundF}) R={distR:F1}m(found={foundR}) center={centerDist:F1}m");

            if (centerDist > 2f)
            {
                // Nudge the train to center the fuel car on the loader.
                // Direction: the fuel car's closer end tells us which way to move.
                bool goForward = foundF && (!foundR || distF < distR);

                Loader.Mod.Logger.Log($"Autopilot RefuelAction: nudging {centerDist:F1}m " +
                    $"(forward={goForward}) to center fuel car on loader");

                _statusMessage = $"Positioning for {_facility.FuelType}...";
                trainService.MoveDistance(loco, centerDist, goForward);
                _phase = Phase.Positioning;
                _statusTimer = 0f;
                return new InProgress();
            }

            // Already close enough — activate loader directly
            return TransitionToRefueling(loco, trainService);
        }

        private ActionOutcome TickPositioning(BaseLocomotive loco, TrainService trainService)
        {
            if (!trainService.IsStoppedForDuration(loco, 0.5f))
                return new InProgress();

            return TransitionToRefueling(loco, trainService);
        }

        private ActionOutcome TransitionToRefueling(BaseLocomotive loco, TrainService trainService)
        {
            // Find and activate the loader sequencer
            var error = FindAndActivateLoader();
            if (error != null)
                return new ActionFailed(error);

            _phase = Phase.Refueling;
            _statusTimer = 0f;
            float level = trainService.GetFuelLevel(loco, _facility.FuelType);
            _statusMessage = $"Refueling {_facility.FuelType} ({level:0}%)...";
            Loader.Mod.Logger.Log($"Autopilot RefuelAction: refueling started, " +
                $"{_facility.FuelType} level={level:F1}%");
            return new InProgress();
        }

        private ActionOutcome TickRefueling(BaseLocomotive loco, TrainService trainService)
        {
            _statusTimer += AutopilotController.TickInterval;

            float level = trainService.GetFuelLevel(loco, _facility.FuelType);

            // Track starting level and progress
            if (_refuelStartLevel < 0f)
                _refuelStartLevel = level;

            // Update status message
            _statusMessage = $"Refueling {_facility.FuelType} ({level:0}%)...";

            // Check if tank is full (within threshold)
            if (level >= 100f - (AutopilotConstants.FullThresholdUnits / GetMaxCapacity(loco, trainService) * 100f))
            {
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: {_facility.FuelType} tank full at {level:F1}%");
                _phase = Phase.Cleanup;
                return new InProgress();
            }

            // Check if loader ran out of supply
            if (IsLoaderEmpty())
            {
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: {_facility.FuelType} loader empty, " +
                    $"tank at {level:F1}%");
                _phase = Phase.Cleanup;
                return new InProgress();
            }

            // Progress check: if no fuel has flowed in 30 seconds, the car
            // probably isn't positioned close enough to the loader.
            if (level > _refuelStartLevel + 1f)
            {
                // Fuel is flowing — reset progress timer
                _noProgressTimer = 0f;
                _refuelStartLevel = level;
            }
            else
            {
                _noProgressTimer += AutopilotController.TickInterval;
                if (_noProgressTimer > 30f)
                {
                    Loader.Mod.Logger.Log($"Autopilot RefuelAction: no fuel progress in 30s " +
                        $"({_facility.FuelType} stuck at {level:F1}%), car may not be positioned correctly");
                    _phase = Phase.Cleanup;
                    return new InProgress();
                }
            }

            // Safety timeout
            if (_statusTimer > 300f)
            {
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: refueling timeout after 300s, " +
                    $"tank at {level:F1}%");
                _phase = Phase.Cleanup;
                return new InProgress();
            }

            return new InProgress();
        }

        private ActionOutcome TickCleanup(BaseLocomotive loco, TrainService trainService)
        {
            DeactivateLoader();

            // Restore original max speed before stopping
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            var helper = new AutoEngineerOrdersHelper(loco, persistence);
            helper.SetOrdersValue(null, null, _previousMaxSpeed, null, null);

            trainService.StopAE(loco);

            float level = trainService.GetFuelLevel(loco, _facility.FuelType);
            Loader.Mod.Logger.Log($"Autopilot RefuelAction: complete, " +
                $"{_facility.FuelType} at {level:F1}%");

            return new ActionDone();
        }

        /// <summary>
        /// Find the CarLoadTargetLoader matching our facility by world position,
        /// then find the matching CarLoaderSequencer and activate it.
        /// Returns an error message if not found, or null on success.
        /// </summary>
        private string? FindAndActivateLoader()
        {
            // Match by RegisteredId — stable across floating-origin shifts.
            // World positions change as the game's coordinate origin moves,
            // but RegisteredId is a persistent unique identifier.
            var loaders = UnityEngine.Object.FindObjectsOfType<CarLoadTargetLoader>();
            CarLoadTargetLoader? matchedLoader = null;

            foreach (var loader in loaders)
            {
                if (loader.keyValueObject.RegisteredId == _facility.LoaderRegisteredId)
                {
                    matchedLoader = loader;
                    break;
                }
            }

            if (matchedLoader == null)
                return $"Cannot find loader for {_facility.FuelType} (id={_facility.LoaderRegisteredId}).";

            // Find matching sequencer by RegisteredId
            var sequencers = UnityEngine.Object.FindObjectsOfType<CarLoaderSequencer>();
            _sequencer = null;
            foreach (var seq in sequencers)
            {
                if (seq.keyValueObject.RegisteredId == matchedLoader.keyValueObject.RegisteredId)
                {
                    _sequencer = seq;
                    break;
                }
            }

            if (_sequencer == null)
                return $"Cannot find loader sequencer for {_facility.FuelType} loader " +
                       $"'{matchedLoader.keyValueObject.RegisteredId}'.";

            // Activate the sequencer
            StateManager.ApplyLocal(new PropertyChange(
                _sequencer.keyValueObject.RegisteredId,
                _sequencer.readWantsLoadingKey,
                new BoolPropertyValue(true)));

            _loaderActivated = true;
            Loader.Mod.Logger.Log($"Autopilot RefuelAction: activated loader sequencer " +
                $"'{_sequencer.keyValueObject.RegisteredId}' for {_facility.FuelType}");

            return null;
        }

        /// <summary>
        /// Deactivate the loader sequencer if we activated it.
        /// </summary>
        private void DeactivateLoader()
        {
            if (_sequencer != null && _loaderActivated)
            {
                StateManager.ApplyLocal(new PropertyChange(
                    _sequencer.keyValueObject.RegisteredId,
                    _sequencer.readWantsLoadingKey,
                    new BoolPropertyValue(false)));

                _loaderActivated = false;
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: deactivated loader sequencer " +
                    $"'{_sequencer.keyValueObject.RegisteredId}'");
            }
        }

        /// <summary>
        /// Check if the industry supplying this loader has run out of the fuel resource.
        /// Water towers (null IndustryId) are considered infinite — always returns false.
        /// </summary>
        private bool IsLoaderEmpty()
        {
            if (_facility.IndustryId == null)
                return false;

            var industry = OpsController.Shared?.IndustryForId(_facility.IndustryId);
            if (industry == null)
                return false;

            if (industry.Storage == null)
            {
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: storage null for " +
                    $"industry '{_facility.IndustryId}'");
                return true;
            }

            var matchingLoad = industry.Storage.Loads()
                .FirstOrDefault(l => l.id == _facility.FuelType);

            if (matchingLoad == null)
            {
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: no matching load " +
                    $"'{_facility.FuelType}' in industry '{_facility.IndustryId}'");
                return true;
            }

            return industry.Storage.QuantityInStorage(matchingLoad) <= 0f;
        }

        /// <summary>
        /// Get the max capacity for the fuel type on the fuel car, for full-threshold calculation.
        /// </summary>
        private float GetMaxCapacity(BaseLocomotive loco, TrainService trainService)
        {
            var fuelCar = trainService.GetFuelCar(loco);
            var slotIndex = fuelCar.Definition.LoadSlots
                .FindIndex(s => s.RequiredLoadIdentifier == _facility.FuelType);
            if (slotIndex < 0)
                return 1f; // avoid divide-by-zero, won't match anyway

            return fuelCar.Definition.LoadSlots[slotIndex].MaximumCapacity;
        }
    }
}
