using System;
using System.Collections.Generic;
using System.Linq;
using Game.Messages;
using Game.State;
using Helpers;
using KeyValue.Runtime;
using Model;
using Model.Ops;
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

        // Loader state — resolved once when we start refueling
        private CarLoaderSequencer? _sequencer;
        private bool _loaderActivated;

        /// <summary>The fuel type this action is refueling.</summary>
        public string FuelType => _facility.FuelType;

        public string StatusMessage => _statusMessage;

        /// <summary>Coupling gap between adjacent cars (from WaypointQueue).</summary>
        private const float CouplingGap = 1.04f;

        public RefuelAction(FacilityInfo facility, BaseLocomotive loco, TrainService trainService)
        {
            _facility = facility;
            _statusMessage = $"Moving to {facility.FuelType} facility...";
            _phase = Phase.MovingToFacility;

            try
            {
                // Compute precise waypoint that positions the fuel car's load slot
                // directly under the loader, using WaypointQueue's approach.
                var preciseLocation = ComputeLoadSlotWaypoint(loco, trainService);
                var waypointDP = Autopilot.Model.DirectedPosition.FromLocation(preciseLocation);

                trainService.SetWaypoint(loco, waypointDP);

                Loader.Mod.Logger.Log($"Autopilot RefuelAction: moving to {facility.FuelType} facility " +
                    $"at {facility.Location.Segment?.id}|{facility.Location.DistanceFromA:F1}, " +
                    $"precise waypoint at {preciseLocation.segment?.id}|{preciseLocation.distance:F1}, " +
                    $"distance={facility.Distance:F0}m");
            }
            catch (Exception ex)
            {
                _initError = $"Failed to set waypoint for refuel: {ex.Message}";
                Loader.Mod.Logger.Log($"Autopilot RefuelAction: {_initError}");
            }
        }

        /// <summary>
        /// Compute the precise waypoint Location that will position the fuel car's
        /// load slot directly under the loader. Ported from WaypointQueue's RefuelService.GetRefuelLocation.
        ///
        /// The AE waypoint targets the FRONT of the train (leading end in direction of travel).
        /// We need to figure out where to put the front of the train so the fuel car's
        /// load slot ends up at the loader's position.
        /// </summary>
        private Location ComputeLoadSlotWaypoint(BaseLocomotive loco, TrainService trainService)
        {
            var graph = Graph.Shared;
            var fuelCar = trainService.GetFuelCar(loco);

            // 1. Find the fuel car's load slot position in game space
            Vector3 loadSlotPosition = GetFuelCarLoadSlotPosition(fuelCar, _facility.FuelType);

            // 2. Get the loader's track location
            Location loaderLocation = _facility.Location.ToLocation();

            // 3. Get the consist's end locations (closest and furthest from the loader)
            var consist = loco.EnumerateCoupled(Car.LogicalEnd.A).ToList();
            var firstEndLoc = consist.First().LocationFor(Car.LogicalEnd.A);
            var lastEndLoc = consist.Last().LocationFor(Car.LogicalEnd.B);

            // Determine which train end is closer to the loader
            float distFirst = Vector3.Distance(firstEndLoc.GetPosition().ZeroY(), loaderLocation.GetPosition().ZeroY());
            float distLast = Vector3.Distance(lastEndLoc.GetPosition().ZeroY(), loaderLocation.GetPosition().ZeroY());

            Location closestTrainEndLocation, furthestTrainEndLocation;
            if (distFirst <= distLast)
            {
                closestTrainEndLocation = firstEndLoc;
                furthestTrainEndLocation = lastEndLoc;
            }
            else
            {
                closestTrainEndLocation = lastEndLoc;
                furthestTrainEndLocation = firstEndLoc;
            }

            // 4. Determine which end of the fuel car is furthest from the loader,
            //    then enumerate cars from that end to calculate distances
            Car.LogicalEnd furthestFuelCarEnd = fuelCar.ClosestLogicalEndTo(furthestTrainEndLocation, graph);
            Car.LogicalEnd closestFuelCarEnd = furthestFuelCarEnd == Car.LogicalEnd.A
                ? Car.LogicalEnd.B : Car.LogicalEnd.A;

            // Get cars from the fuel car toward the furthest train end (inclusive)
            var carsTowardFurthestEnd = EnumerateAdjacentCarsTowardEnd(fuelCar, furthestFuelCarEnd);
            float distFromFurthestEndToFuelCar = CalculateTotalLength(carsTowardFurthestEnd);

            // Distance from the fuel car's closest end to its load slot
            float distFromClosestEndToSlot = Vector3.Distance(
                fuelCar.LocationFor(closestFuelCarEnd).GetPosition().ZeroY(),
                loadSlotPosition.ZeroY());

            float totalTrainLength = CalculateTotalLength(consist);

            Loader.Mod.Logger.Log($"Autopilot RefuelAction: positioning — " +
                $"totalTrainLen={totalTrainLength:F1}, " +
                $"distFurthestToFuelCar={distFromFurthestEndToFuelCar:F1}, " +
                $"distClosestEndToSlot={distFromClosestEndToSlot:F1}");

            // 5. Calculate the offset using IsTargetBetween geometry checks
            //    (same logic as WaypointQueue)
            Location locationToMoveToward = new();
            float distanceToMove = 0;

            if (IsTargetBetween(loadSlotPosition, loaderLocation.GetPosition(), closestTrainEndLocation.GetPosition()))
            {
                // Slot is between loader and closest end — move toward far end
                distanceToMove = distFromFurthestEndToFuelCar - distFromClosestEndToSlot;
                locationToMoveToward = furthestTrainEndLocation;
            }

            if (IsTargetBetween(loadSlotPosition, loaderLocation.GetPosition(), furthestTrainEndLocation.GetPosition()))
            {
                // Slot is between loader and furthest end — move toward near end
                distanceToMove = totalTrainLength - distFromFurthestEndToFuelCar + distFromClosestEndToSlot;
                locationToMoveToward = closestTrainEndLocation;
            }

            if (IsTargetBetween(loaderLocation.GetPosition(), closestTrainEndLocation.GetPosition(), furthestTrainEndLocation.GetPosition()))
            {
                // Loader is between the two train ends — negate
                distanceToMove = -distanceToMove;
            }

            Loader.Mod.Logger.Log($"Autopilot RefuelAction: computed offset={distanceToMove:F2}m");

            // 6. Apply offset: orient the loader location toward the direction we want to move,
            //    then move by the computed distance
            Location oriented = graph.LocationOrientedToward(loaderLocation, locationToMoveToward);
            Location result = graph.LocationByMoving(oriented, distanceToMove, true, true);

            return result;
        }

        /// <summary>
        /// Get the game-space position of the fuel car's load slot for the given fuel type.
        /// Ported from WaypointQueue's GetFuelCarLoadSlotPosition + CalculatePositionFromLoadTarget.
        /// </summary>
        private static Vector3 GetFuelCarLoadSlotPosition(Car fuelCar, string fuelType)
        {
            var loadSlot = fuelCar.Definition.LoadSlots.Find(slot => slot.RequiredLoadIdentifier == fuelType);
            if (loadSlot == null)
                throw new InvalidOperationException($"Fuel car {fuelCar.DisplayName} has no load slot for '{fuelType}'");

            int slotIndex = fuelCar.Definition.LoadSlots.IndexOf(loadSlot);
            var carLoadTargets = fuelCar.GetComponentsInChildren<CarLoadTarget>().ToList();
            var loadTarget = carLoadTargets.Find(clt => clt.slotIndex == slotIndex);
            if (loadTarget == null)
                throw new InvalidOperationException($"Fuel car {fuelCar.DisplayName} has no CarLoadTarget for slot index {slotIndex}");

            // Convert load target's local position to game space using the car's transform matrix
            // (same logic as CarLoadTargetLoader.LoadSlotFromCar)
            Matrix4x4 transformMatrix = fuelCar.GetTransformMatrix(Graph.Shared);
            Vector3 localPoint = fuelCar.transform.InverseTransformPoint(loadTarget.transform.position);
            Vector3 gamePosition = transformMatrix.MultiplyPoint3x4(localPoint);
            return gamePosition;
        }

        /// <summary>
        /// Check if a target point lies between two other points in XZ space.
        /// If the target is in the middle, its distance to either endpoint is less than
        /// the distance between the endpoints.
        /// Ported from WaypointQueue's IsTargetBetween.
        /// </summary>
        private static bool IsTargetBetween(Vector3 target, Vector3 positionA, Vector3 positionB)
        {
            float distAToTarget = Vector3.Distance(positionA.ZeroY(), target.ZeroY());
            float distBToTarget = Vector3.Distance(positionB.ZeroY(), target.ZeroY());
            float distAToB = Vector3.Distance(positionA.ZeroY(), positionB.ZeroY());
            return distAToTarget < distAToB && distBToTarget < distAToB;
        }

        /// <summary>
        /// Calculate total length of a list of cars including coupling gaps.
        /// Uses CouplingGap (1.04f) per coupling, matching WaypointQueue's CalculateTotalLength.
        /// </summary>
        private static float CalculateTotalLength(List<Car> cars)
        {
            float length = 0f;
            foreach (var car in cars)
                length += car.carLength;
            return length + CouplingGap * (cars.Count - 1);
        }

        /// <summary>
        /// Enumerate cars from a starting car toward a specific logical end, inclusive.
        /// Returns the starting car plus all cars coupled in that direction.
        /// Ported from WaypointQueue's carService.EnumerateAdjacentCarsTowardEnd.
        /// </summary>
        private static List<Car> EnumerateAdjacentCarsTowardEnd(Car startCar, Car.LogicalEnd towardEnd)
        {
            var result = new List<Car>();
            var current = startCar;
            var currentEnd = towardEnd;

            while (current != null)
            {
                result.Add(current);
                var next = current.CoupledTo(currentEnd);
                if (next == null)
                    break;
                // Figure out which end of the next car connects back to current
                var nextEndBack = next.CoupledTo(Car.LogicalEnd.A) == current
                    ? Car.LogicalEnd.A : Car.LogicalEnd.B;
                // Continue toward the opposite end of the next car
                currentEnd = nextEndBack == Car.LogicalEnd.A ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                current = next;
            }

            return result;
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

            // The precise waypoint computed in the constructor should have positioned
            // the fuel car's load slot directly at the loader. Go straight to refueling.
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

            // Check if tank is full (percentage threshold — avoids the old
            // absolute-units calculation that varied wildly by tank size)
            if (level >= AutopilotConstants.FullThresholdPercent)
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
            if (level > _refuelStartLevel)
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

    }
}
