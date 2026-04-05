using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;
using UI;
using UI.Builder;
using UI.Common;
using Model;
using Cameras;
using Autopilot.Model;
using Autopilot.Execution;
using Autopilot.Planning;
using Autopilot.Services;

namespace Autopilot.UI
{
    public class AutopilotWindow
    {
        private Window _window;
        private UIPanel _panel;
        private bool _isOpen;
        private AutopilotMode _selectedMode = AutopilotMode.Delivery;
        private PickupFilter _pickupFilter = PickupFilter.Default;
        private List<string> _fromOptions = new List<string>();
        private List<string> _toOptions = new List<string>();
        private PickupPlanner? _pickupPlanner;
        private int _eligibleCarCount;
        private bool _deliverAfterPickup = false;

        // Event-driven rebuild tracking
        private AutopilotStateMachine _subscribedSm;
        private string _lastLocoId;
        private bool _lastHadWaypoint;
        private string _lastStatusMessage;
        private string _lastErrorMessage;
        private Coroutine _ticker;
        private float _scrollPosition;
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
            if (!_isOpen || _panel == null) return;

            // Save scroll position before rebuild
            var scrollRect = _window.contentRectTransform.GetComponentInChildren<ScrollRect>();
            if (scrollRect != null)
                _scrollPosition = scrollRect.verticalNormalizedPosition;

            _rebuildMethod?.Invoke(_panel, null);

            // Restore scroll position after layout reflows (next frame)
            AutopilotController.Instance?.StartCoroutine(RestoreScrollPosition());
        }

        private IEnumerator RestoreScrollPosition()
        {
            yield return null;
            var scrollRect = _window?.contentRectTransform?.GetComponentInChildren<ScrollRect>();
            if (scrollRect != null)
                scrollRect.verticalNormalizedPosition = _scrollPosition;
        }

        private void OnStateChanged()
        {
            RebuildPanel();
        }

        private void RefreshPickupOptions(BaseLocomotive loco)
        {
            if (_pickupPlanner == null)
                _pickupPlanner = new PickupPlanner(TrainSvc);

            // No cross-filtering: each axis shows all options based on distance + base filters only.
            // Preserve the user's selected mode on the axis we're populating so ExtractGroupingKeys
            // knows which keys to extract, but set the opposite axis to Any.
            if (_pickupFilter.From.Mode != Autopilot.Model.FilterMode.Any && _pickupFilter.From.Mode != Autopilot.Model.FilterMode.Switchlist)
            {
                var fromFilter = new PickupFilter(_pickupFilter.From, FilterAxis.Any, _pickupFilter.MaxDistance, false);
                _fromOptions = _pickupPlanner.GetFromOptions(loco, fromFilter);
            }
            else
                _fromOptions.Clear();

            if (_pickupFilter.To.Mode != Autopilot.Model.FilterMode.Any && _pickupFilter.To.Mode != Autopilot.Model.FilterMode.Switchlist)
            {
                var toFilter = new PickupFilter(FilterAxis.Any, _pickupFilter.To, _pickupFilter.MaxDistance, false);
                _toOptions = _pickupPlanner.GetToOptions(loco, toFilter);
            }
            else
                _toOptions.Clear();

            _pickupFilter.From.CheckedItems.IntersectWith(_fromOptions);
            _pickupFilter.To.CheckedItems.IntersectWith(_toOptions);

            // Eligible count still uses the full filter (both axes) to show actual matches
            _eligibleCarCount = _pickupPlanner.GetEligibleCars(loco, _pickupFilter).Count;
        }

        private string BuildFilterSummary(PickupFilter filter, int carCount)
        {
            // "Picking up 5 car(s) within 2.1 mi on the switchlist going to Bryson, Andrews"
            var parts = new List<string>();
            parts.Add($"Picking up {carCount} car(s)");

            // Distance
            if (filter.MaxDistance < float.MaxValue)
            {
                float feet = filter.MaxDistance * 3.281f;
                string dist = feet >= 5280f ? $"{(feet / 5280f):F1} mi" : $"{feet:F0} ft";
                parts.Add($"within {dist}");
            }

            // From
            if (filter.From.Mode != Autopilot.Model.FilterMode.Any)
            {
                if (filter.From.Mode == Autopilot.Model.FilterMode.Switchlist)
                    parts.Add("on the switchlist");
                else if (filter.From.CheckedItems.Count > 0)
                {
                    string label = filter.From.Mode switch
                    {
                        Autopilot.Model.FilterMode.Area => "in",
                        Autopilot.Model.FilterMode.Industry => "at",
                        Autopilot.Model.FilterMode.Destination => "at",
                        _ => "from"
                    };
                    parts.Add($"{label} {string.Join(", ", filter.From.CheckedItems.OrderBy(s => s))}");
                }
            }

            // To
            if (filter.To.Mode != Autopilot.Model.FilterMode.Any)
            {
                if (filter.To.Mode == Autopilot.Model.FilterMode.Switchlist)
                    parts.Add("on the switchlist");
                else if (filter.To.CheckedItems.Count > 0)
                    parts.Add($"going to {string.Join(", ", filter.To.CheckedItems.OrderBy(s => s))}");
            }

            return string.Join(" ", parts) + ".";
        }

        private List<string> GetFilterModeLabels()
        {
            string switchlistLabel = "On switchlist";
            var crewName = TrainSvc.GetTrainCrewName();
            if (!string.IsNullOrEmpty(crewName))
                switchlistLabel = $"On switchlist ({crewName})";

            return new List<string>
            {
                "Any",
                "Area",
                "Industry",
                "Destination",
                switchlistLabel
            };
        }

        private void BuildPickupFilterUI(UIPanelBuilder builder, BaseLocomotive loco, PickupFilter filter,
            Func<bool> getDeliverAfter = null, Action<bool> setDeliverAfter = null)
        {
            var loco2 = loco; // capture for lambdas

            // Distance slider — getValue reads live from filter so it doesn't snap back during drag.
            // Game units are meters; display in feet/miles. 1 meter ≈ 3.281 feet.
            float sliderMaxMeters = 72000f; // ~45 miles
            var filterRef = filter; // capture for lambdas
            builder.AddSlider(
                () => filterRef.MaxDistance >= sliderMaxMeters ? sliderMaxMeters : filterRef.MaxDistance,
                () =>
                {
                    if (filterRef.MaxDistance >= sliderMaxMeters) return "\u221E";
                    float feet = filterRef.MaxDistance * 3.281f;
                    return feet >= 5280f ? $"{(feet / 5280f):F1} mi" : $"{feet:F0} ft";
                },
                (val) => { filterRef.MaxDistance = val >= sliderMaxMeters ? float.MaxValue : val; },
                0f, sliderMaxMeters);

            // From/To columns side by side
            var filterModeLabels = GetFilterModeLabels();

            builder.HStack(hstack =>
            {
                // FROM column
                hstack.VStack(fromCol =>
                {
                    fromCol.FieldLabelWidth = 140f;
                    fromCol.AddLabel("<b>From</b>");
                    int fromIdx = (int)filter.From.Mode;
                    fromCol.AddDropdown(filterModeLabels, fromIdx, (idx) =>
                    {
                        filter.From = new FilterAxis((Autopilot.Model.FilterMode)idx, new HashSet<string>());
                        RefreshPickupOptions(loco2);
                        RebuildPanel();
                    });

                    if (filter.From.Mode != Autopilot.Model.FilterMode.Any && filter.From.Mode != Autopilot.Model.FilterMode.Switchlist)
                    {
                        foreach (var option in _fromOptions)
                        {
                            var opt = option;
                            fromCol.AddField(opt, fromCol.AddToggle(
                                () => filter.From.CheckedItems.Contains(opt),
                                (val) =>
                                {
                                    if (val)
                                        filter.From.CheckedItems.Add(opt);
                                    else
                                        filter.From.CheckedItems.Remove(opt);
                                    RefreshPickupOptions(loco2);
                                    RebuildPanel();
                                }));
                        }
                        if (_fromOptions.Count == 0)
                            fromCol.AddLabel("<color=yellow>No options</color>");
                    }
                });

                // TO column
                hstack.VStack(toCol =>
                {
                    toCol.FieldLabelWidth = 140f;
                    toCol.AddLabel("<b>To</b>");
                    int toIdx = (int)filter.To.Mode;
                    toCol.AddDropdown(filterModeLabels, toIdx, (idx) =>
                    {
                        filter.To = new FilterAxis((Autopilot.Model.FilterMode)idx, new HashSet<string>());
                        RefreshPickupOptions(loco2);
                        RebuildPanel();
                    });

                    if (filter.To.Mode != Autopilot.Model.FilterMode.Any && filter.To.Mode != Autopilot.Model.FilterMode.Switchlist)
                    {
                        foreach (var option in _toOptions)
                        {
                            var opt = option;
                            toCol.AddField(opt, toCol.AddToggle(
                                () => filter.To.CheckedItems.Contains(opt),
                                (val) =>
                                {
                                    if (val)
                                        filter.To.CheckedItems.Add(opt);
                                    else
                                        filter.To.CheckedItems.Remove(opt);
                                    RefreshPickupOptions(loco2);
                                    RebuildPanel();
                                }));
                        }
                        if (_toOptions.Count == 0)
                            toCol.AddLabel("<color=yellow>No options</color>");
                    }
                });
            });

            // Options row
            builder.HStack(row =>
            {
                row.AddField("Auto-add to switchlist", row.AddToggle(
                    () => filter.AutoAddToSwitchlist,
                    (val) => filter.AutoAddToSwitchlist = val));
                if (getDeliverAfter != null && setDeliverAfter != null)
                {
                    row.AddField("Deliver after pickup", row.AddToggle(getDeliverAfter, setDeliverAfter));
                }
            });

            // Summary
            RefreshPickupOptions(loco);
            var summary = BuildFilterSummary(filter, _eligibleCarCount);
            if (_eligibleCarCount == 0)
                builder.AddLabel($"<color=yellow>{summary}</color>");
            else
                builder.AddLabel(summary);
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
            _window.SetContentSize(new Vector2Int(500, 500));
            _window.SetResizable(new Vector2(500, 500), new Vector2(650, Screen.height));
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

        private void BuildContent(UIPanelBuilder outerBuilder)
        {
            var controller = AutopilotController.Instance;
            if (controller == null)
            {
                outerBuilder.AddLabel("Autopilot controller not initialized.");
                return;
            }

            outerBuilder.VScrollView(builder =>
            {
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
                        () => { _selectedMode = AutopilotMode.Delivery; RebuildPanel(); });
                    strip.AddButtonSelectable("Pickup",
                        _selectedMode == AutopilotMode.Pickup,
                        () =>
                        {
                            if (_selectedMode != AutopilotMode.Pickup)
                            {
                                _selectedMode = AutopilotMode.Pickup;
                                _pickupFilter = PickupFilter.Default;
                                RefreshPickupOptions(loco);
                                RebuildPanel();
                            }
                        });
                });

                if (_selectedMode == AutopilotMode.Pickup)
                {
                    BuildPickupFilterUI(builder, loco, _pickupFilter,
                        () => _deliverAfterPickup, (val) => _deliverAfterPickup = val);
                }
            }

            // Show filter controls and deliver-after-pickup toggle during pickup execution
            if (sm != null && sm.Mode == AutopilotMode.Pickup && sm.Phase is not (Idle or Completed))
            {
                // Sync UI filter state from state machine so edits go to the live filter
                if (sm.PickupFilter != null)
                    _pickupFilter = sm.PickupFilter;

                BuildPickupFilterUI(builder, loco, _pickupFilter,
                    () => sm.DeliverAfterPickup, (val) => sm.DeliverAfterPickup = val);
            }

            // Control buttons
            bool autopilotRunning = sm != null && sm.Phase is not (Idle or Completed or Failed);

            builder.ButtonStrip(strip =>
            {
                if (sm == null || sm.Phase is Idle or Completed)
                {
                    if (_selectedMode == AutopilotMode.Pickup)
                    {
                        if (_eligibleCarCount > 0)
                        {
                            strip.AddButton("Start Pickup", () =>
                            {
                                controller.StartPickup(_pickupFilter, _deliverAfterPickup);
                                RebuildPanel();
                            });
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

            // Refuel controls on their own row
            builder.ButtonStrip(strip =>
            {
                strip.AddButton("Refuel Now", () =>
                {
                    var controller = AutopilotController.Instance;
                    if (controller != null)
                    {
                        var currentSm = controller.GetStateMachineForSelected();
                        if (currentSm != null && currentSm.Phase is not (Idle or Completed or Failed))
                        {
                            // Autopilot is running — request refuel on next planning tick
                            currentSm.RequestRefuel();
                        }
                        else
                        {
                            // Autopilot is idle — start a refuel-only run
                            controller.StartRefuel();
                        }
                        RebuildPanel();
                    }
                }).Disable(sm != null && sm.Phase is Executing e && e.CurrentAction is RefuelAction)
                  .Tooltip("Refuel Now", "Route to the nearest fuel facility and refuel immediately");
            });

            builder.AddFieldToggle("Refuel when low",
                () => TrainSvc.GetAutoRefuelEnabled(loco),
                (val) => TrainSvc.SetAutoRefuelEnabled(loco, val))
                .Tooltip("Auto-Refuel",
                    "Automatically refuel when fuel/water drops below threshold (configure in mod settings)");

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
                builder.AddLabel($"Collecting cars for: <b>{sm.PickupFilter?.DisplaySummary ?? "?"}</b>");
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
            });
        }
    }
}
