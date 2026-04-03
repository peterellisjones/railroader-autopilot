using System.Collections.Generic;
using System.Linq;
using Game.Messages;
using KeyValue.Runtime;
using Model;
using Model.AI;
using Model.Ops;
using System.Reflection;
using Track;
using UI.EngineControls;
using HarmonyLib;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Services
{
    public class TrainService
    {
        private const string ParkingSpaceKey = "autopilot.parkingSpace";
        private const string ParkAfterDeliveryKey = "autopilot.parkAfterDelivery";
        private const string ActiveKey = "autopilot.active";
        private const string ModeKey = "autopilot.mode";
        private const string TargetDestinationKey = "autopilot.targetDestination";
        private const string PickupCountKey = "autopilot.pickupCount";
        private const string ContextKey = "autopilot.context";
        private const string DeliverAfterPickupKey = "autopilot.deliverAfterPickup";

        // Per-plan-cycle caches — cleared by DeliveryPlanner.BuildPlan()
        private List<Car>? _cachedCoupled;
        private string? _cachedCoupledLocoId;
        private float? _cachedTrainLength;
        private string? _cachedTrainLengthLocoId;

        public void ClearPlanCaches()
        {
            _cachedCoupled = null;
            _cachedCoupledLocoId = null;
            _cachedTrainLength = null;
            _cachedTrainLengthLocoId = null;
        }

        public List<Car> GetCoupled(BaseLocomotive loco)
        {
            if (_cachedCoupled != null && _cachedCoupledLocoId == loco.id)
                return _cachedCoupled;
            _cachedCoupled = loco.EnumerateCoupled(Car.LogicalEnd.A).ToList();
            _cachedCoupledLocoId = loco.id;
            return _cachedCoupled;
        }

        public Waybill? GetWaybill(Car car)
        {
            return car.Waybill;
        }

        public void SetWaypoint(BaseLocomotive loco, DirectedPosition target)
        {
            var location = target.ToLocation();
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            var helper = new AutoEngineerOrdersHelper(loco, persistence);
            helper.SetOrdersValue(AutoEngineerMode.Waypoint, null, Loader.Settings.maxSpeedMph, null, (location, null));

            // Update the waypoint overlay so the marker appears on the track.
            // The overlay controller lives on the locomotive's GameObject.
            var overlayController = loco.GetComponentInChildren<AutoEngineerWaypointOverlayController>();
            if (overlayController != null)
            {
                OrderWaypoint? waypoint = new OrderWaypoint(Graph.Shared.LocationToString(location), null);
                overlayController.WaypointDidChange(waypoint);
            }
        }

        public void SetWaypointWithCouple(BaseLocomotive loco, DirectedPosition target, string coupleToCarId)
        {
            var location = target.ToLocation();
            var locationString = Graph.Shared.LocationToString(location);
            Loader.Mod.Logger.Log($"Autopilot SetWaypointWithCouple: locationString={locationString}, carId={coupleToCarId}");

            // SetOrdersValue with Waypoint mode — same as WaypointQueue and the
            // game's destination picker. Explicitly set Waypoint mode so the AE
            // responds even if the loco was in Road/Yard/Off mode.
            // Do NOT also call HandleAutoEngineerCommand, as SetOrdersValue
            // internally triggers it. A second call interrupts the route planning.
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            var helper = new AutoEngineerOrdersHelper(loco, persistence);
            helper.SetOrdersValue(AutoEngineerMode.Waypoint, null, Loader.Settings.maxSpeedMph, null, (location.Clamped(), coupleToCarId));

            var orders = persistence.Orders;
            Loader.Mod.Logger.Log($"Autopilot SetWaypointWithCouple: after set: mode={orders.Mode}, hasWaypoint={orders.Waypoint.HasValue}, speed={orders.MaxSpeedMph}, plannerStatus={persistence.PlannerStatus}");

            var overlayController = loco.GetComponentInChildren<AutoEngineerWaypointOverlayController>()
                ?? UnityEngine.Object.FindObjectOfType<AutoEngineerWaypointOverlayController>();
            if (overlayController != null)
            {
                OrderWaypoint? waypoint = new OrderWaypoint(locationString, coupleToCarId);
                overlayController.WaypointDidChange(waypoint);
            }
        }

        public void StopAE(BaseLocomotive loco)
        {
            var command = new AutoEngineerCommand
            {
                LocomotiveId = loco.id,
                Mode = AutoEngineerMode.Off
            };
            TrainController.Shared.HandleAutoEngineerCommand(command, null);
        }

        public void ClearWaypoint(BaseLocomotive loco)
        {
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            var helper = new AutoEngineerOrdersHelper(loco, persistence);
            helper.ClearWaypoint();

            var overlayController = loco.GetComponentInChildren<AutoEngineerWaypointOverlayController>();
            if (overlayController != null)
                overlayController.WaypointDidChange(null);
        }

        public void MoveDistance(BaseLocomotive loco, float distance, bool? forward = null)
        {
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            var helper = new AutoEngineerOrdersHelper(loco, persistence);
            helper.SetOrdersValue(AutoEngineerMode.Yard, forward, null, distance);
        }

        public Orders GetOrders(BaseLocomotive loco)
        {
            var persistence = new AutoEngineerPersistence(loco.KeyValueObject);
            return persistence.Orders;
        }

        /// <summary>
        /// Returns the LocationString of the loco's current active waypoint, or null
        /// if the loco is not in waypoint mode or has no waypoint set.
        /// </summary>
        public string? GetCurrentWaypointLocationString(BaseLocomotive loco)
        {
            var orders = GetOrders(loco);
            if (orders.Mode == AutoEngineerMode.Waypoint && orders.Waypoint.HasValue)
                return orders.Waypoint.Value.LocationString;
            return null;
        }

        /// <summary>
        /// Saves a location string as this locomotive's parking space.
        /// Persists in the game save via KeyValueObject.
        /// </summary>
        public void SaveParkingSpace(BaseLocomotive loco, string locationString)
        {
            loco.KeyValueObject[ParkingSpaceKey] = Value.String(locationString);
        }

        /// <summary>
        /// Returns the saved parking space location string, or null if none is set.
        /// </summary>
        public string? GetParkingSpace(BaseLocomotive loco)
        {
            var value = loco.KeyValueObject[ParkingSpaceKey];
            if (value.IsNull)
                return null;
            return value.StringValue;
        }

        /// <summary>
        /// Sets the loco's waypoint to its saved parking space.
        /// Returns false if no parking space is saved.
        /// </summary>
        public bool GoToParkingSpace(BaseLocomotive loco)
        {
            var locationString = GetParkingSpace(loco);
            if (locationString == null)
                return false;

            Location location;
            try
            {
                location = Graph.Shared.ResolveLocationString(locationString);
            }
            catch
            {
                Loader.Mod.Logger.Log($"Autopilot: Saved parking space is no longer valid, clearing.");
                loco.KeyValueObject[ParkingSpaceKey] = Value.Null();
                return false;
            }

            var directedPosition = DirectedPosition.FromLocation(location);
            SetWaypoint(loco, directedPosition);
            return true;
        }

        public bool GetParkAfterDelivery(BaseLocomotive loco)
        {
            var value = loco.KeyValueObject[ParkAfterDeliveryKey];
            return !value.IsNull && value.BoolValue;
        }

        public void SetParkAfterDelivery(BaseLocomotive loco, bool enabled)
        {
            loco.KeyValueObject[ParkAfterDeliveryKey] = Value.Bool(enabled);
        }

        public void SaveAutopilotState(BaseLocomotive loco, AutopilotMode mode,
            string? targetDestination, int pickupCount, PlanningContext context,
            bool deliverAfterPickup)
        {
            loco.KeyValueObject[ActiveKey] = Value.Bool(true);
            loco.KeyValueObject[ModeKey] = Value.String(mode.ToString());
            loco.KeyValueObject[TargetDestinationKey] = targetDestination != null
                ? Value.String(targetDestination) : Value.Null();
            loco.KeyValueObject[PickupCountKey] = Value.Int(pickupCount);
            loco.KeyValueObject[ContextKey] = Value.String(context.Serialize());
            loco.KeyValueObject[DeliverAfterPickupKey] = Value.Bool(deliverAfterPickup);
        }

        public void ClearAutopilotState(BaseLocomotive loco)
        {
            loco.KeyValueObject[ActiveKey] = Value.Bool(false);
            loco.KeyValueObject[ModeKey] = Value.Null();
            loco.KeyValueObject[TargetDestinationKey] = Value.Null();
            loco.KeyValueObject[PickupCountKey] = Value.Null();
            loco.KeyValueObject[ContextKey] = Value.Null();
            loco.KeyValueObject[DeliverAfterPickupKey] = Value.Null();
        }

        public SavedAutopilotState? LoadAutopilotState(BaseLocomotive loco)
        {
            var activeVal = loco.KeyValueObject[ActiveKey];
            if (activeVal.IsNull || !activeVal.BoolValue)
                return null;

            var modeVal = loco.KeyValueObject[ModeKey];
            var mode = AutopilotMode.Delivery;
            if (!modeVal.IsNull && modeVal.StringValue == AutopilotMode.Pickup.ToString())
                mode = AutopilotMode.Pickup;

            var destVal = loco.KeyValueObject[TargetDestinationKey];
            string? targetDestination = destVal.IsNull ? null : destVal.StringValue;

            var pickupVal = loco.KeyValueObject[PickupCountKey];
            int pickupCount = pickupVal.IsNull ? 0 : pickupVal.IntValue;

            var contextVal = loco.KeyValueObject[ContextKey];
            PlanningContext context;
            if (contextVal.IsNull || string.IsNullOrEmpty(contextVal.StringValue))
            {
                context = PlanningContext.Empty;
            }
            else
            {
                try
                {
                    context = PlanningContext.Deserialize(contextVal.StringValue);
                }
                catch
                {
                    Loader.Mod.Logger.Log($"Autopilot: Failed to deserialize context for {loco.DisplayName}, using empty context.");
                    context = PlanningContext.Empty;
                }
            }

            var deliverAfterVal = loco.KeyValueObject[DeliverAfterPickupKey];
            bool deliverAfterPickup = !deliverAfterVal.IsNull && deliverAfterVal.BoolValue;

            return new SavedAutopilotState(mode, targetDestination, pickupCount, context, deliverAfterPickup);
        }

        public bool IsWaypointMode(BaseLocomotive loco)
        {
            return GetOrders(loco).Mode == AutoEngineerMode.Waypoint;
        }

        public bool IsWaypointSatisfied(BaseLocomotive loco)
        {
            // When the AE reaches a waypoint, it clears the waypoint from orders
            // but stays in Waypoint mode. So: waypoint mode + no active waypoint = arrived.
            // This is the same pattern used by WaypointQueue.
            var orders = GetOrders(loco);
            return orders.Mode == AutoEngineerMode.Waypoint && !orders.Waypoint.HasValue;
        }

        public bool IsStopped(BaseLocomotive loco)
        {
            return loco.VelocityMphAbs < 0.1f;
        }

        public bool IsStoppedForDuration(BaseLocomotive loco, float seconds)
        {
            return loco.IsStopped(seconds);
        }

        public void Uncouple(Car car, Car.LogicalEnd end)
        {
            var adjacent = car.CoupledTo(end);
            if (adjacent == null)
                return;

            // Find which end of the adjacent car actually faces this car
            var adjEnd = adjacent.CoupledTo(Car.LogicalEnd.A) == car
                ? Car.LogicalEnd.A
                : Car.LogicalEnd.B;

            var carEndGear = end == Car.LogicalEnd.A ? car.EndGearA : car.EndGearB;
            var adjEndGear = adjEnd == Car.LogicalEnd.A ? adjacent.EndGearA : adjacent.EndGearB;

            // Only close angle cocks and disconnect air if air was connected
            if (carEndGear.IsAirConnected || adjEndGear.IsAirConnected)
            {
                car.ApplyEndGearChange(end, Car.EndGearStateKey.Anglecock, 0f);
                adjacent.ApplyEndGearChange(adjEnd, Car.EndGearStateKey.Anglecock, 0f);
                car.ApplyEndGearChange(end, Car.EndGearStateKey.IsAirConnected, false);
                adjacent.ApplyEndGearChange(adjEnd, Car.EndGearStateKey.IsAirConnected, false);
            }

            // Pull the cut lever
            adjacent.ApplyEndGearChange(adjEnd, Car.EndGearStateKey.CutLever, 1f);
        }

        /// <summary>
        /// Tell the AE to refresh its consist after cars have been coupled/uncoupled.
        /// Without this, the AE still thinks the old cars are attached and can't
        /// plan routes correctly. Same pattern as WaypointQueue.
        /// </summary>
        public void UpdateCarsForAE(BaseLocomotive loco)
        {
            var updateCars = AccessTools.Method(typeof(AutoEngineerPlanner), "UpdateCars");
            updateCars?.Invoke(loco.AutoEngineerPlanner, new object[] { null });
        }

        public void SetHandbrake(Car car, bool on)
        {
            car.SetHandbrake(on);
        }

        public void DisconnectAir(Car car, Car.LogicalEnd end)
        {
            car.ApplyEndGearChange(end, Car.EndGearStateKey.Anglecock, 0f);
        }

        public void ConnectAirOnCoupled(BaseLocomotive loco)
        {
            foreach (var car in loco.EnumerateCoupled(Car.LogicalEnd.A))
            {
                // Open angle cocks and connect air on both ends
                if (car.CoupledTo(Car.LogicalEnd.A) != null)
                {
                    car.ApplyEndGearChange(Car.LogicalEnd.A, Car.EndGearStateKey.Anglecock, 1f);
                    car.ApplyEndGearChange(Car.LogicalEnd.A, Car.EndGearStateKey.IsAirConnected, true);
                }
                if (car.CoupledTo(Car.LogicalEnd.B) != null)
                {
                    car.ApplyEndGearChange(Car.LogicalEnd.B, Car.EndGearStateKey.Anglecock, 1f);
                    car.ApplyEndGearChange(Car.LogicalEnd.B, Car.EndGearStateKey.IsAirConnected, true);
                }
            }
        }

        public void BleedAir(Car car)
        {
            car.air.BleedBrakeCylinder();
        }

        public bool IsCoupled(Car car, Car.LogicalEnd end)
        {
            var endGear = end == Car.LogicalEnd.A ? car.EndGearA : car.EndGearB;
            return endGear.IsCoupled;
        }

        public float GetConsistLength(List<Car> cars)
        {
            float length = 0f;
            foreach (var car in cars)
                length += car.carLength;
            if (cars.Count > 1)
                length += (cars.Count - 1) * AutopilotConstants.ConsistGapPerCar;
            return length;
        }

        public BaseLocomotive GetSelectedLocomotive()
        {
            return TrainController.Shared.SelectedLocomotive;
        }

        public float GetTrainLength(BaseLocomotive loco)
        {
            if (_cachedTrainLength.HasValue && _cachedTrainLengthLocoId == loco.id)
                return _cachedTrainLength.Value;
            var cars = GetCoupled(loco);
            float length = 0f;
            foreach (var car in cars)
                length += car.carLength;
            if (cars.Count > 1)
                length += (cars.Count - 1) * AutopilotConstants.ConsistGapPerCar;
            _cachedTrainLength = length;
            _cachedTrainLengthLocoId = loco.id;
            return length;
        }

        /// <summary>
        /// Route distance from the loco to a location, checking both directions
        /// and accounting for blocking cars. Uses RouteSearch with checkForCars=true
        /// (TryFindDistance ignores all cars). Checks from both LocationF and
        /// LocationR since RouteSearch exits the starting segment in one direction only.
        /// Returns null if no route found in either direction.
        /// </summary>
        public RouteResult? GraphDistanceToLoco(BaseLocomotive loco, DirectedPosition target)
        {
            var coupled = GetCoupled(loco);
            var targetLoc = target.ToLocation();
            var resultF = RouteChecker.RouteDistanceWithCars(loco.LocationF, targetLoc, coupled);
            var resultR = RouteChecker.RouteDistanceWithCars(loco.LocationR, targetLoc, coupled);

            if (resultF == null) return resultR;
            if (resultR == null) return resultF;
            return resultF.Value.Distance <= resultR.Value.Distance ? resultF : resultR;
        }

        /// <summary>
        /// Returns all cars in the world NOT coupled to the given loco.
        /// </summary>
        public List<Car> GetNearbyCars(BaseLocomotive loco)
        {
            var coupled = new HashSet<Car>(GetCoupled(loco));
            var result = new List<Car>();

            foreach (var car in TrainController.Shared.Cars)
            {
                if (!coupled.Contains(car) && car.LocationA.segment != null
                    && car.gameObject.activeInHierarchy)
                    result.Add(car);
            }

            return result;
        }
    }
}
