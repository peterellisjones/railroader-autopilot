using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UI;
using UI.Builder;
using UI.Common;
using Model;
using Cameras;
using Autopilot.Model;
using Autopilot.Execution;
using Autopilot.Services;

namespace Autopilot.UI
{
    public class AutopilotWindow
    {
        private Window _window;
        private UIPanel _panel;
        private bool _isOpen;
        private AutopilotMode _selectedMode = AutopilotMode.Delivery;
        private int _selectedDestinationIndex = -1;
        private List<string> _destinations = new List<string>();

        // Event-driven rebuild tracking
        private AutopilotStateMachine _subscribedSm;
        private string _lastLocoId;
        private bool _lastHadWaypoint;
        private string _lastStatusMessage;
        private string _lastErrorMessage;
        private Coroutine _ticker;
        private TrainService _trainService;
        private TrainService TrainSvc => _trainService ??= new TrainService();

        private static readonly MethodInfo _rebuildMethod =
            typeof(UIPanel).GetMethod("Rebuild", BindingFlags.Instance | BindingFlags.NonPublic);

        public void Toggle()
        {
            if (_isOpen)
                Close();
            else
                Open();
        }

        public void Open()
        {
            if (_window != null)
            {
                _window.ShowWindow();
                _isOpen = true;
                StartTicker();
                return;
            }

            CreateWindow();
            _isOpen = true;
            StartTicker();
        }

        public void Close()
        {
            if (_window != null)
                _window.CloseWindow();
            _isOpen = false;
            StopTicker();
        }

        private void StartTicker()
        {
            StopTicker();
            var controller = AutopilotController.Instance;
            if (controller != null)
                _ticker = controller.StartCoroutine(Ticker());
        }

        private void StopTicker()
        {
            if (_ticker != null && AutopilotController.Instance != null)
            {
                AutopilotController.Instance.StopCoroutine(_ticker);
                _ticker = null;
            }
        }

        /// <summary>
        /// Polling coroutine: checks for loco selection changes and
        /// status updates during execution. Runs while the window is open.
        /// Unlike RebuildOnInterval, this only rebuilds when something
        /// actually changed, so dropdowns are never interrupted.
        /// </summary>
        private IEnumerator Ticker()
        {
            var wait = new WaitForSeconds(0.5f);
            while (_isOpen)
            {
                yield return wait;

                var currentLocoId = TrainController.Shared?.SelectedLocomotive?.id;
                if (currentLocoId != _lastLocoId)
                {
                    _lastLocoId = currentLocoId;
                    RebuildPanel();
                    continue;
                }

                // Rebuild when AE mode changes (for parking buttons).
                // Check mode rather than waypoint string — mode is stable,
                // whereas the waypoint can briefly toggle during AE route planning.
                var loco = TrainController.Shared?.SelectedLocomotive;
                bool inWaypointMode = loco != null && TrainSvc.IsWaypointMode(loco);
                if (inWaypointMode != _lastHadWaypoint)
                {
                    _lastHadWaypoint = inWaypointMode;
                    RebuildPanel();
                    continue;
                }

                // During execution, rebuild only when status text actually changed
                var sm = AutopilotController.Instance?.GetStateMachineForSelected();
                if (sm != null && sm.Phase is Executing)
                {
                    if (sm.StatusMessage != _lastStatusMessage || sm.ErrorMessage != _lastErrorMessage)
                    {
                        _lastStatusMessage = sm.StatusMessage;
                        _lastErrorMessage = sm.ErrorMessage;
                        RebuildPanel();
                    }
                }
            }

            _ticker = null;
        }

        private void RebuildPanel()
        {
            if (_isOpen && _panel != null)
                _rebuildMethod?.Invoke(_panel, null);
        }

        private void OnStateChanged()
        {
            RebuildPanel();
        }

        private void CreateWindow()
        {
            var creator = UnityEngine.Object.FindObjectOfType<ProgrammaticWindowCreator>();
            if (creator == null)
            {
                Loader.Mod.Logger.Log("Autopilot: Cannot find ProgrammaticWindowCreator.");
                return;
            }

            // CreateWindow() is not publicly accessible — use reflection
            var createMethod = typeof(ProgrammaticWindowCreator)
                .GetMethod("CreateWindow", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance, null, Type.EmptyTypes, null);
            if (createMethod != null)
            {
                _window = (Window)createMethod.Invoke(creator, null);
            }
            else
            {
                // Try instantiating from the windowPrefab field
                var prefabField = typeof(ProgrammaticWindowCreator)
                    .GetField("windowPrefab", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                if (prefabField != null)
                {
                    var prefab = (Window)prefabField.GetValue(creator);
                    if (prefab != null)
                    {
                        _window = UnityEngine.Object.Instantiate(prefab);
                    }
                }
            }

            if (_window == null)
            {
                Loader.Mod.Logger.Log("Autopilot: Cannot create window.");
                return;
            }

            _window.Title = "Autopilot";
            _window.SetContentSize(new Vector2Int(340, 400));
            _window.SetPosition(Window.Position.LowerRight);

            // builderAssets may be private — try field access, fall back to reflection
            UIBuilderAssets assets = null;
            var assetsField = typeof(ProgrammaticWindowCreator)
                .GetField("builderAssets", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (assetsField != null)
            {
                assets = (UIBuilderAssets)assetsField.GetValue(creator);
            }

            if (assets == null)
            {
                Loader.Mod.Logger.Log("Autopilot: Cannot access builderAssets.");
                return;
            }

            _panel = UIPanel.Create(
                _window.contentRectTransform,
                assets,
                BuildContent
            );

            _window.OnShownDidChange += (shown) =>
            {
                _isOpen = shown;
                if (shown)
                    StartTicker();
                else
                    StopTicker();
            };

            _window.ShowWindow();
        }

        private void BuildContent(UIPanelBuilder builder)
        {
            var controller = AutopilotController.Instance;
            if (controller == null)
            {
                builder.AddLabel("Autopilot controller not initialized.");
                return;
            }

            builder.Spacing = 1f;

            var loco = TrainController.Shared?.SelectedLocomotive;
            _lastLocoId = loco?.id;
            builder.AddLabel($"<b>{(loco != null ? loco.DisplayName : "No locomotive selected")}</b>");

            if (loco == null)
                return;

            var sm = controller.GetStateMachineForSelected();

            // Subscribe to state changes for event-driven rebuilds.
            if (sm != _subscribedSm)
            {
                if (_subscribedSm != null)
                    _subscribedSm.OnStateChanged -= OnStateChanged;
                _subscribedSm = sm;
                if (sm != null)
                    sm.OnStateChanged += OnStateChanged;
            }

            bool isIdle = sm == null || sm.Phase is Idle or Completed;

            if (isIdle)
            {
                builder.ButtonStrip(strip =>
                {
                    strip.AddButtonSelectable("Delivery",
                        _selectedMode == AutopilotMode.Delivery,
                        () => { _selectedMode = AutopilotMode.Delivery; _selectedDestinationIndex = -1; RebuildPanel(); });
                    strip.AddButtonSelectable("Pickup",
                        _selectedMode == AutopilotMode.Pickup,
                        () =>
                        {
                            if (_selectedMode != AutopilotMode.Pickup)
                            {
                                _selectedMode = AutopilotMode.Pickup;
                                _destinations = controller.GetReachableDestinations();
                                _selectedDestinationIndex = _destinations.Count > 0 ? 0 : -1;
                                RebuildPanel();
                            }
                        });
                });

                if (_selectedMode == AutopilotMode.Pickup)
                {
                    if (_destinations.Count == 0)
                    {
                        builder.AddLabel("<color=yellow>No reachable cars with waybills found.</color>");
                    }
                    else
                    {
                        builder.AddField("Destination",
                            builder.AddDropdown(
                                _destinations,
                                _selectedDestinationIndex >= 0 ? _selectedDestinationIndex : 0,
                                (index) => { _selectedDestinationIndex = index; }
                            )
                        );
                    }
                }
            }

            // Control buttons
            bool autopilotRunning = sm != null && sm.Phase is not (Idle or Completed or Failed);

            builder.ButtonStrip(strip =>
            {
                if (sm == null || sm.Phase is Idle or Completed)
                {
                    if (_selectedMode == AutopilotMode.Pickup)
                    {
                        bool canStart = _destinations.Count > 0 && _selectedDestinationIndex >= 0;
                        if (canStart)
                        {
                            strip.AddButton("Start Pickup", () =>
                                { controller.StartPickup(_destinations[_selectedDestinationIndex]); RebuildPanel(); });
                        }
                    }
                    else
                    {
                        strip.AddButton("Start", () => { controller.StartAutopilot(); RebuildPanel(); });
                    }
                }
                else if (sm.Phase is Failed)
                {
                    strip.AddButton("Retry", () => { controller.RetryAutopilot(); RebuildPanel(); });
                    strip.AddButton("Abort", () => { controller.StopAutopilot(); RebuildPanel(); });
                }
                else
                {
                    strip.AddButton("Stop", () => { controller.StopAutopilot(); RebuildPanel(); });
                }

                // Jump to the AE's actual live waypoint, not the mod's internal target
                var wpString = TrainSvc.GetCurrentWaypointLocationString(loco);
                bool hasActiveWaypoint = wpString != null;
                strip.AddButton("Jump to WP", () =>
                {
                    var currentWp = TrainSvc.GetCurrentWaypointLocationString(loco);
                    if (currentWp != null)
                    {
                        var loc = Track.Graph.Shared.ResolveLocationString(currentWp);
                        CameraSelector.shared.JumpToPoint(
                            loc.GetPosition(),
                            loc.GetRotation(),
                            CameraSelector.CameraIdentifier.Strategy);
                    }
                }).Disable(!hasActiveWaypoint);
            });

            // Parking buttons on their own row
            var parkWpString = TrainSvc.GetCurrentWaypointLocationString(loco);
            bool parkHasWaypoint = parkWpString != null;
            bool hasParkingSpace = TrainSvc.GetParkingSpace(loco) != null;

            builder.ButtonStrip(strip =>
            {
                bool canSetPark = parkHasWaypoint && !autopilotRunning;
                strip.AddButton("Set Park", () =>
                {
                    var wp = TrainSvc.GetCurrentWaypointLocationString(loco);
                    if (wp != null)
                    {
                        TrainSvc.SaveParkingSpace(loco, wp);
                        RebuildPanel();
                    }
                }).Disable(!canSetPark)
                  .Tooltip("Parking Space",
                      autopilotRunning ? "Stop autopilot first"
                      : !parkHasWaypoint ? "Set a waypoint first, then save it as a parking space"
                      : "Save the current waypoint as this loco's parking space");

                bool canPark = hasParkingSpace && !autopilotRunning;
                strip.AddButton("Park", () =>
                {
                    TrainSvc.GoToParkingSpace(loco);
                    RebuildPanel();
                }).Disable(!canPark)
                  .Tooltip("Parking Space",
                      autopilotRunning ? "Stop autopilot first"
                      : !hasParkingSpace ? "No parking space saved \u2014 use Set Park first"
                      : "Send this loco to its saved parking space");
            });

            // "Park after delivery" checkbox
            if (TrainSvc.GetParkingSpace(loco) != null)
            {
                builder.AddFieldToggle("Park after delivery",
                    () => TrainSvc.GetParkAfterDelivery(loco),
                    (val) => TrainSvc.SetParkAfterDelivery(loco, val))
                    .Tooltip("Park After Delivery",
                        "Automatically send this loco to its parking space when deliveries are complete");
            }

            if (sm == null)
                return;

            // Status
            if (!string.IsNullOrEmpty(sm.StatusMessage))
                builder.AddLabel(sm.StatusMessage);

            if (!string.IsNullOrEmpty(sm.ErrorMessage))
                builder.AddLabel($"<color=red>{sm.ErrorMessage}</color>");

            // Pickup progress
            if (sm.Mode == AutopilotMode.Pickup)
            {
                builder.AddLabel($"Collecting cars for: <b>{sm.TargetDestination ?? "?"}</b>");
                if (sm.PickupCount > 0)
                    builder.AddLabel($"Cars collected: {sm.PickupCount}");
            }

            // Plan view — delivery mode only
            if (sm.Mode == AutopilotMode.Delivery)
            {
                var plan = sm.Plan;
                if (plan != null)
                {
                    if (plan.NeedsRunaround)
                    {
                        builder.AddLabel($"\u21c4  Runaround to {plan.Runaround.CoupleTarget.DisplayName}");
                    }

                    if (plan.HasDeliveries)
                    {
                        const int MaxVisibleLines = 3;
                        builder.AddSection("Deliveries");
                        int shownPending = 0;
                        int pendingLines = 0;
                        int i = 0;
                        while (i < plan.Steps.Count)
                        {
                            var step = plan.Steps[i];
                            bool isPending = step.Status == StepStatus.Pending;

                            // Non-pending steps (done/in-progress/error) always shown individually
                            if (!isPending)
                            {
                                string icon = step.Status switch
                                {
                                    StepStatus.Done => "\u2713",
                                    StepStatus.InProgress => "\u25BA",
                                    StepStatus.Error => "\u2717",
                                    _ => "\u25CB"
                                };
                                var carNames = string.Join(", ", step.Cars.Select(c => c.DisplayName));
                                builder.AddLabel($"{icon}  {carNames}  \u2192  {step.DestinationName}");
                                i++;
                                continue;
                            }

                            if (shownPending >= MaxVisibleLines)
                            {
                                i++;
                                pendingLines++;
                                continue;
                            }

                            // Group consecutive pending steps with the same destination
                            int totalCars = step.Cars.Count;
                            int groupEnd = i + 1;
                            while (groupEnd < plan.Steps.Count
                                && plan.Steps[groupEnd].Status == StepStatus.Pending
                                && plan.Steps[groupEnd].DestinationName == step.DestinationName)
                            {
                                totalCars += plan.Steps[groupEnd].Cars.Count;
                                groupEnd++;
                            }

                            if (groupEnd == i + 1)
                            {
                                // Single step — show car names
                                var carNames = string.Join(", ", step.Cars.Select(c => c.DisplayName));
                                builder.AddLabel($"\u25CB  {carNames}  \u2192  {step.DestinationName}");
                            }
                            else
                            {
                                // Multiple steps same dest — show count
                                builder.AddLabel($"\u25CB  {totalCars} car(s)  \u2192  {step.DestinationName}");
                            }

                            shownPending++;
                            pendingLines += groupEnd - i;
                            i = groupEnd;
                        }

                        int totalPending = plan.Steps.Count(s => s.Status == StepStatus.Pending);
                        int hiddenPending = totalPending - pendingLines;
                        if (hiddenPending > 0)
                            builder.AddLabel($"     +{hiddenPending} more");
                    }

                    if (plan.Warnings.Count > 0)
                    {
                        builder.AddSection("Notes");
                        foreach (var warning in plan.Warnings)
                            builder.AddLabel($"<color=yellow>{warning}</color>");
                    }
                }
            }
        }
    }
}
