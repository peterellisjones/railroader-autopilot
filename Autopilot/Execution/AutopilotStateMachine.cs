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

        public string? TargetDestination => Phase switch
        {
            PlanningPhase p => p.TargetDestination,
            Executing e => e.TargetDestination,
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
        }

        public void Start(BaseLocomotive loco, AutopilotMode mode = AutopilotMode.Delivery, string? destinationName = null)
        {
            _loco = loco;

            if (_pickupPlanner == null)
                _pickupPlanner = new PickupPlanner(_trainService);

            // Release handbrakes and connect air on the whole consist.
            foreach (var car in _trainService.GetCoupled(loco))
                _trainService.SetHandbrake(car, false);
            _trainService.ConnectAirOnCoupled(loco);

            SetPhase(new PlanningPhase(PlanningContext.Empty, mode, destinationName));
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
                savedState.TargetDestination,
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
                _retryDestination,
                _retryPickupCount));
        }

        // Retry state preserved when entering Failed
        private PlanningContext? _retryContext;
        private AutopilotMode _retryMode;
        private string? _retryDestination;
        private int _retryPickupCount;

        public void Tick()
        {
            if (_loco == null)
                return;

            // Clear per-plan-cycle caches so execution actions see fresh
            // coupling state. The caches are still useful within BuildPlan
            // (which runs inside a single tick).
            _trainService.ClearPlanCaches();

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
                        pickupCount += pa.TargetCarCount;

                    SetPhase(new PlanningPhase(context, exec.Mode, exec.TargetDestination, pickupCount));
                    return;

                case ActionFailed failed:
                    Loader.Mod.Logger.Log($"Autopilot: action failed: {failed.Reason}");
                    EnterFailed(failed.Reason, exec);
                    return;
            }
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
                // If there are dropped cars pending recouple, go get them first
                if (p.Context.PendingSplit != null)
                {
                    var split = p.Context.PendingSplit;
                    var newContext = p.Context.WithPendingSplit(null);
                    var action = new RecoupleAction(split, _loco, _trainService);
                    SetPhase(new Executing(plan, action, newContext, p.Mode, p.TargetDestination, p.PickupCount, split.CoupleLocation));
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

            if (plan.HasDeliveries)
            {
                var step = plan.Steps[0];
                Loader.Mod.Logger.Log($"Autopilot: action=Deliver {step.Cars.Count} car(s) to {step.DestinationName}" +
                    $" (couple={step.CoupleTarget?.DisplayName ?? "none"})");
                var newContext = p.Context.WithClearedSwitches();
                var action = new DeliveryAction(step, _loco, _trainService);
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.TargetDestination, p.PickupCount, step.DestinationLocation));
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
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.TargetDestination, p.PickupCount, split.CoupleLocation));
                return;
            }

            if (plan.NeedsRunaround)
            {
                Loader.Mod.Logger.Log($"Autopilot: action=Runaround, split={plan.Runaround.SplitCar.DisplayName}, " +
                    $"couple={plan.Runaround.CoupleTarget.DisplayName}, reason={plan.Reason}");
                var action = new RunaroundExecutionAction(plan.Runaround, _loco, _trainService);
                SetPhase(new Executing(plan, action, p.Context, p.Mode, p.TargetDestination, p.PickupCount, plan.Runaround.CoupleLocation));
                return;
            }

            if (plan.NeedsSplit)
            {
                Loader.Mod.Logger.Log($"Autopilot: action=Split, drop {plan.Split.DroppedCars.Count} car(s), " +
                    $"keep tail for {plan.Reason}");
                var newContext = p.Context.WithPendingSplit(plan.Split);
                var action = new SplitAction(plan.Split, _loco, _trainService);
                SetPhase(new Executing(plan, action, newContext, p.Mode, p.TargetDestination, p.PickupCount, null));
                return;
            }

            // Reposition to a loop
            Loader.Mod.Logger.Log($"Autopilot: action=Reposition, reason={plan.Reason}");
            var repositionContext = p.Context;
            if (plan.RepositionLoopKey != null)
                repositionContext = repositionContext.WithVisitedLoop(plan.RepositionLoopKey);
            var repositionAction = new RepositionAction(
                plan.RepositionLocation!.Value, _loco, _trainService, plan.Reason);
            SetPhase(new Executing(plan, repositionAction, repositionContext, p.Mode, p.TargetDestination, p.PickupCount, plan.RepositionLocation.Value));
        }

        private void TickPickupPlanning(PlanningPhase p)
        {
            PickupTarget? target;
            try
            {
                target = _pickupPlanner!.FindNextPickup(_loco, p.TargetDestination, p.Context.SkippedCars);
            }
            catch (Exception ex)
            {
                Loader.Mod.Logger.Log($"Autopilot: pickup planning failed: {ex}");
                EnterFailed($"Pickup planning failed: {ex.Message}", p);
                return;
            }

            if (target == null)
            {
                if (DeliverAfterPickup)
                {
                    Loader.Mod.Logger.Log($"Autopilot: Pickup complete ({p.PickupCount} cars) — starting delivery");
                    SetPhase(new PlanningPhase(PlanningContext.Empty, AutopilotMode.Delivery, null));
                    return;
                }
                var msg = $"Pickup complete — {p.PickupCount} car(s) collected for {p.TargetDestination}.";
                SetPhase(new Completed(msg, p.PickupCount));
                return;
            }

            var action = new PickupAction(target, _loco, _trainService);
            SetPhase(new Executing(null, action, p.Context, p.Mode, p.TargetDestination, p.PickupCount, target.CoupleLocation));
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
                    _retryDestination = pp.TargetDestination;
                    _retryPickupCount = pp.PickupCount;
                    break;
                case Executing ex:
                    _retryContext = ex.Context;
                    _retryMode = ex.Mode;
                    _retryDestination = ex.TargetDestination;
                    _retryPickupCount = ex.PickupCount;
                    break;
                default:
                    _retryContext = PlanningContext.Empty;
                    _retryMode = AutopilotMode.Delivery;
                    _retryDestination = null;
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
                        _trainService.SaveAutopilotState(_loco, p.Mode, p.TargetDestination, p.PickupCount, p.Context, DeliverAfterPickup);
                        break;
                    case Executing e:
                        _trainService.SaveAutopilotState(_loco, e.Mode, e.TargetDestination, e.PickupCount, e.Context, DeliverAfterPickup);
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
