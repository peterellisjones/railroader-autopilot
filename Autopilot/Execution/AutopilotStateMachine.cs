using System;
using System.Collections.Generic;
using Model;
using Autopilot.Model;
using Autopilot.Planning;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class AutopilotStateMachine
    {
        private readonly TrainService _trainService;
        private readonly DeliveryPlanner _planner;
        private BaseLocomotive _loco;
        private PickupPlanner? _pickupPlanner;
        private readonly RefuelPlanner _refuelPlanner;
        private bool _refuelRequested;
        private string? _lastRefuelFuelType;

        public AutopilotPhase Phase { get; private set; } = new Idle();

        // Convenience accessors for UI compatibility
        public DeliveryPlan? Plan => Phase is Executing e ? e.Plan : null;
        public string StatusMessage => Phase.StatusMessage;
        public string? ErrorMessage => Phase is Failed f ? f.ErrorMessage : null;
        public DirectedPosition? CurrentWaypoint => Phase is Executing e ? e.CurrentWaypoint : null;

        public AutopilotMode Mode => Phase switch
        {
            PlanningPhase p => p.Mode,
            Executing e => e.Mode,
            _ => AutopilotMode.Delivery
        };

        public PickupFilter? PickupFilter => Phase switch
        {
            PlanningPhase p => p.PickupFilter,
            Executing e => e.PickupFilter,
            _ => null
        };

        public int PickupCount => Phase switch
        {
            PlanningPhase p => p.PickupCount,
            Executing e => e.PickupCount,
            Completed c => c.PickupCount,
            _ => 0
        };

        public bool DeliverAfterPickup { get; set; }

        public event Action? OnStateChanged;

        public AutopilotStateMachine(TrainService trainService)
        {
            _trainService = trainService;
            _planner = new DeliveryPlanner(trainService);
            _refuelPlanner = new RefuelPlanner(trainService);
        }

        public void Start(BaseLocomotive loco, AutopilotMode mode = AutopilotMode.Delivery, PickupFilter? pickupFilter = null)
        {
            _loco = loco;

            if (_pickupPlanner == null)
                _pickupPlanner = new PickupPlanner(_trainService);

            // Release handbrakes and connect air on the whole consist.
            foreach (var car in _trainService.GetCoupled(loco))
                _trainService.SetHandbrake(car, false);
            _trainService.ConnectAirOnCoupled(loco);

            SetPhase(new PlanningPhase(PlanningContext.Empty, mode, pickupFilter));
        }

        public void Resume(BaseLocomotive loco, SavedAutopilotState savedState)
        {
            _loco = loco;

            if (_pickupPlanner == null)
                _pickupPlanner = new PickupPlanner(_trainService);

            DeliverAfterPickup = savedState.DeliverAfterPickup;

            SetPhase(new PlanningPhase(
                savedState.Context,
                savedState.Mode,
                savedState.PickupFilter,
                savedState.PickupCount));
        }

        public void Stop()
        {
            if (_loco != null)
                _trainService.ClearWaypoint(_loco);
            SetPhase(new Idle());
        }

        public void Retry()
        {
            if (Phase is not Failed)
                return;

            // Preserve mode, context, destination, pickup count from the failed phase.
            // We need to recover these from the Failed phase — but Failed only stores
            // ErrorMessage and LastPlan. The mode/context/destination are lost.
            // We need to re-enter planning with the last known context.
            // Since Failed doesn't carry context, we restart with empty context.
            // This matches the old behavior: Retry() just set state to Planning
            // with whatever _context was (which was never cleared on error).
            //
            // To preserve context across errors, we store it in a field when entering Failed.
            SetPhase(new PlanningPhase(
                _retryContext ?? PlanningContext.Empty,
                _retryMode,
                _retryPickupFilter,
                _retryPickupCount));
        }

        /// <summary>
        /// Request an immediate refuel on the next planning tick.
        /// Called by the "Refuel Now" button.
        /// </summary>
        public void RequestRefuel()
        {
            _refuelRequested = true;
        }

        // Retry state preserved when entering Failed
        private PlanningContext? _retryContext;
        private AutopilotMode _retryMode;
        private PickupFilter? _retryPickupFilter;
        private int _retryPickupCount;

        public void Tick()
        {
            if (_loco == null)
                return;

            // Clear per-plan-cycle caches so execution actions see fresh
            // coupling state. The caches are still useful within BuildPlan
            // (which runs inside a single tick).
            _trainService.ClearPlanCaches();
            _refuelPlanner.ClearCache();

            switch (Phase)
            {
                case Executing exec:
                    TickExecuting(exec);
                    break;
                case PlanningPhase p:
                    if (p.Mode == AutopilotMode.Pickup)
                        TickPickupPlanning(p);
                    else
                        TickPlanning(p);
                    break;
            }
        }

        private void TickExecuting(Executing exec)
        {
            var outcome = exec.CurrentAction.Tick(_loco, _trainService);

            switch (outcome)
            {
                case InProgress:
                    return;

                case ActionDone:
                case ActionReplan replan:
                    var context = exec.Context;
                    var pickupCount = exec.PickupCount;

                    if (outcome is ActionReplan r && r.SkippedCars != null)
                    {
                        context = context.WithSkippedCars(r.SkippedCars);
                        Loader.Mod.Logger.Log($"Autopilot: {r.SkippedCars.Count} car(s) skipped (siding full), total skipped: {context.SkippedCars.Count}");
                    }

                    if (exec.Mode == AutopilotMode.Pickup && exec.CurrentAction is PickupAction pa)
                    {
                        pickupCount += pa.TargetCarCount;

                        // Auto-add to switchlist after coupling
                        if (exec.PickupFilter?.AutoAddToSwitchlist == true)
                        {
                            var carIds = pa.PickedUpCarIds;
                            if (carIds.Count > 0)
                                _trainService.AddToSwitchlist(carIds);
                        }
                    }

                    // Track last refueled fuel type for opportunistic check
                    if (exec.CurrentAction is RefuelAction refuelAction)
                        _lastRefuelFuelType = refuelAction.FuelType;
                    else
                        _lastRefuelFuelType = null;

                    SetPhase(new PlanningPhase(context, exec.Mode, exec.PickupFilter, pickupCount));
                    return;

                case ActionFailed failed:
                    Loader.Mod.Logger.Log($"Autopilot: action failed: {failed.Reason}");
                    EnterFailed(failed.Reason, exec);
                    return;
            }
        }

        /// <summary>
        /// Check if the loco needs refueling and create a RefuelAction if so.
        /// Returns the action if refueling is needed, null otherwise.
        /// </summary>
        private RefuelAction? TryCreateRefuelAction(BaseLocomotive loco, int thresholdPercent)
        {
            if (!_trainService.GetAutoRefuelEnabled(loco) && !_refuelRequested)
                return null;

            int threshold = _refuelRequested ? 100 : thresholdPercent;
            if (threshold <= 0 && !_refuelRequested)
                return null;

            // After an opportunistic refuel, check for nearby second fuel type
            if (_lastRefuelFuelType != null)
            {
                var nearby = _refuelPlanner.FindNearbyOpportunistic(loco, _lastRefuelFuelType);
                _lastRefuelFuelType = null;
                if (nearby != null)
                {
                    Loader.Mod.Logger.Log($"Autopilot: opportunistic refuel {nearby.FuelType} at nearby facility");
                    return new RefuelAction(nearby, loco, _trainService);
                }
            }

            var lowTypes = _refuelPlanner.GetLowFuelTypes(loco, threshold);
            if (lowTypes.Count == 0)
            {
                if (_refuelRequested)
                {
                    _refuelRequested = false;
                    Loader.Mod.Logger.Log("Autopilot: Refuel requested but all fuel types are sufficient.");
                }
                return null;
            }

            var facility = _refuelPlanner.FindBestFacility(loco, lowTypes);
            if (facility == null)
            {
                var typeStr = string.Join(", ", lowTypes);
                Loader.Mod.Logger.Log($"Autopilot: No reachable facility for {typeStr}");
                _refuelRequested = false;
                return null;
            }

            _refuelRequested = false;
            Loader.Mod.Logger.Log($"Autopilot: action=Refuel {facility.FuelType} at {facility.Location.Segment?.id}, dist={facility.Distance:F0}m");
            return new RefuelAction(facility, loco, _trainService);
        }

        private void TickPlanning(PlanningPhase p)
        {
            DeliveryPlan plan;
            try
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                plan = _planner.BuildPlan(_loco, p.Context.VisitedSwitches, p.Context.VisitedLoopKeys, p.Context.SkippedCars);
                sw.Stop();
                if (sw.ElapsedMilliseconds > 10)
                    Loader.Mod.Logger.Log($"Autopilot: BuildPlan took {sw.ElapsedMilliseconds}ms");
            }
            catch (Exception ex)
            {
                Loader.Mod.Logger.Log($"Autopilot: planning failed: {ex}");
                EnterFailed($"Planning failed: {ex.Message}", p);
                return;
            }

            if (!plan.HasDeliveries && !plan.NeedsRunaround && !plan.NeedsReposition && !plan.NeedsSplit)
            {
                // Completion refuel check: before parking, check with higher threshold
                if (p.Context.PendingSplit == null)
                {
                    var completionRefuelAction = TryCreateRefuelAction(_loco, Loader.Settings.completionRefuelThreshold);
                    if (completionRefuelAction != null)
                    {
                        SetPhase(new Executing(plan, completionRefuelAction, p.Context, p.Mode, p.PickupFilter, p.PickupCount, null));
                        return;
                    }
                }

                // If there are dropped cars pending recouple, go get them first
                if (p.Context.PendingSplit != null)
                {
                    var split = p.Context.PendingSplit;
                    var newContext = p.Context.WithPendingSplit(null);
                    var action = new RecoupleAction(split, _loco, _trainService);
                    SetPhase(new Executing(plan, action, newContext, p.Mode, p.PickupFilter, p.PickupCount, split.CoupleLocation.ToDirectedPosition()));
                    return;
                }

                if (plan.Warnings.Count == 0 && p.Context.SkippedCars.Count == 0)
                {
                    if (_trainService.GetParkAfterDelivery(_loco) && _trainService.GoToParkingSpace(_loco))
                        SetPhase(new Completed("All deliveries complete! Parking..."));
                    else
                        SetPhase(new Completed("All deliveries complete!"));
                }
                else if (p.Context.SkippedCars.Count > 0)
                    EnterFailed($"{p.Context.SkippedCars.Count} car(s) skipped — siding(s) full. Stop and restart autopilot to retry.", p);
                else
                    EnterFailed("No deliverable cars found. " + string.Join(" ", plan.Warnings), p);
                return;
            }

            // Mid-run refuel check: before next delivery/action, check if fuel is low
            var midRunRefuelAction = TryCreateRefuelAction(_loco, Loader.Settings.midRunRefuelThreshold);
            if (midRunRefuelAction != null)
            {
                SetPhase(new Executing(plan, midRunRefuelAction, p.Context, p.Mode, p.PickupFilter, p.PickupCount, null));
                return;
            }

            if (plan.HasDeliveries)
            {
                var step = plan.Steps[0];
                Loader.Mod.Logger.Log($"Autopilot: action=Deliver {step.Cars.Count} car(s) to {step.DestinationName}" +
                    $" (couple={step.CoupleTarget?.DisplayName ?? "none"})");
                var newContext = p.Context.WithClearedSwitches();
                var action = new DeliveryAction(step, _loco, _trainService);
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.PickupFilter, p.PickupCount, step.DestinationLocation));
                return;
            }

            // If there are dropped cars pending, recouple BEFORE attempting
            // runaround, split, or reposition.
            if (p.Context.PendingSplit != null)
            {
                var split = p.Context.PendingSplit;
                Loader.Mod.Logger.Log($"Autopilot: action=Recouple {split.DroppedCars.Count} dropped car(s)");
                var newContext = p.Context.WithPendingSplit(null);
                var action = new RecoupleAction(split, _loco, _trainService);
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.PickupFilter, p.PickupCount, split.CoupleLocation.ToDirectedPosition()));
                return;
            }

            if (plan.NeedsRunaround)
            {
                Loader.Mod.Logger.Log($"Autopilot: action=Runaround, split={plan.Runaround.SplitCar.DisplayName}, " +
                    $"couple={plan.Runaround.CoupleTarget.DisplayName}, reason={plan.Reason}");
                var action = new RunaroundExecutionAction(plan.Runaround, _loco, _trainService);
                SetPhase(new Executing(plan, action, p.Context, p.Mode, p.PickupFilter, p.PickupCount, plan.Runaround.CoupleLocation.ToDirectedPosition()));
                return;
            }

            if (plan.NeedsSplit)
            {
                Loader.Mod.Logger.Log($"Autopilot: action=Split, drop {plan.Split.DroppedCars.Count} car(s), " +
                    $"keep tail for {plan.Reason}");
                var newContext = p.Context.WithPendingSplit(plan.Split);
                var action = new SplitAction(plan.Split, _loco, _trainService);
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.PickupFilter, p.PickupCount, null));
                return;
            }

            // Reposition to a loop. Don't mark the loop as visited yet —
            // the reposition may not fully place the train on the loop
            // (e.g. long train tail extends past the stem). Let the next
            // planning cycle re-evaluate the same loop. It will either
            // succeed at the runaround or reposition again.
            Loader.Mod.Logger.Log($"Autopilot: action=Reposition, reason={plan.Reason}");
            var repositionAction = new RepositionAction(
                plan.RepositionLocation!.Value, _loco, _trainService, plan.Reason);
            SetPhase(new Executing(plan, repositionAction, p.Context, p.Mode, p.PickupFilter, p.PickupCount, plan.RepositionLocation.Value));
        }

        private void TickPickupPlanning(PlanningPhase p)
        {
            // Mid-run refuel check for pickup mode
            var pickupRefuelAction = TryCreateRefuelAction(_loco, Loader.Settings.midRunRefuelThreshold);
            if (pickupRefuelAction != null)
            {
                SetPhase(new Executing(null, pickupRefuelAction, p.Context, p.Mode, p.PickupFilter, p.PickupCount, null));
                return;
            }

            PickupTarget? target;
            try
            {
                target = _pickupPlanner!.FindNextPickup(_loco, p.PickupFilter!, p.Context.SkippedCars);
            }
            catch (Exception ex)
            {
                Loader.Mod.Logger.Log($"Autopilot: pickup planning failed: {ex}");
                EnterFailed($"Pickup planning failed: {ex.Message}", p);
                return;
            }

            if (target == null)
            {
                // Completion refuel check before finishing pickup mode
                var pickupCompletionRefuel = TryCreateRefuelAction(_loco, Loader.Settings.completionRefuelThreshold);
                if (pickupCompletionRefuel != null)
                {
                    SetPhase(new Executing(null, pickupCompletionRefuel, p.Context, p.Mode, p.PickupFilter, p.PickupCount, null));
                    return;
                }

                if (DeliverAfterPickup)
                {
                    Loader.Mod.Logger.Log($"Autopilot: Pickup complete ({p.PickupCount} cars) — starting delivery");
                    SetPhase(new PlanningPhase(PlanningContext.Empty, AutopilotMode.Delivery, null));
                    return;
                }
                var msg = $"Pickup complete — {p.PickupCount} car(s) collected.";
                SetPhase(new Completed(msg, p.PickupCount));
                return;
            }

            var action = new PickupAction(target, _loco, _trainService);
            SetPhase(new Executing(null, action, p.Context, p.Mode, p.PickupFilter, p.PickupCount, target.CoupleLocation.ToDirectedPosition()));
        }

        private void EnterFailed(string message, AutopilotPhase fromPhase)
        {
            DeliveryPlan? lastPlan = fromPhase is Executing e ? e.Plan : null;

            if (lastPlan?.Steps?.Count > 0)
                lastPlan.Steps[0].Status = StepStatus.Error;

            // Preserve state for retry
            switch (fromPhase)
            {
                case PlanningPhase pp:
                    _retryContext = pp.Context;
                    _retryMode = pp.Mode;
                    _retryPickupFilter = pp.PickupFilter;
                    _retryPickupCount = pp.PickupCount;
                    break;
                case Executing ex:
                    _retryContext = ex.Context;
                    _retryMode = ex.Mode;
                    _retryPickupFilter = ex.PickupFilter;
                    _retryPickupCount = ex.PickupCount;
                    break;
                default:
                    _retryContext = PlanningContext.Empty;
                    _retryMode = AutopilotMode.Delivery;
                    _retryPickupFilter = null;
                    _retryPickupCount = 0;
                    break;
            }

            SetPhase(new Failed(message, lastPlan));
        }

        private void SetPhase(AutopilotPhase newPhase)
        {
            Phase = newPhase;

            // Persist state for save/load resume
            if (_loco != null)
            {
                switch (newPhase)
                {
                    case PlanningPhase p:
                        _trainService.SaveAutopilotState(_loco, p.Mode, p.PickupFilter, p.PickupCount, p.Context, DeliverAfterPickup);
                        break;
                    case Executing e:
                        _trainService.SaveAutopilotState(_loco, e.Mode, e.PickupFilter, e.PickupCount, e.Context, DeliverAfterPickup);
                        break;
                    default:
                        // Idle, Completed, Failed — clear saved state so we don't resume
                        _trainService.ClearAutopilotState(_loco);
                        break;
                }
            }

            OnStateChanged?.Invoke();
        }
    }
}
