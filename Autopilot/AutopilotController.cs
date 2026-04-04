using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using GalaSoft.MvvmLight.Messaging;
using Game.Events;
using Model;
using Autopilot.Execution;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.Planning;

namespace Autopilot
{
    public class AutopilotController : MonoBehaviour
    {
        public const float TickInterval = 0.5f;

        private TrainService _trainService;
        private Dictionary<BaseLocomotive, AutopilotStateMachine> _stateMachines = new Dictionary<BaseLocomotive, AutopilotStateMachine>();
        private Dictionary<BaseLocomotive, Coroutine> _coroutines = new Dictionary<BaseLocomotive, Coroutine>();
        private PickupPlanner _pickupPlanner;

        public static AutopilotController Instance { get; private set; }

        private void Awake()
        {
            Instance = this;
            _trainService = new TrainService();
        }

        private void OnDestroy()
        {
            if (Instance == this)
                Instance = null;
        }

        private void OnEnable()
        {
            Messenger.Default.Register<PropertiesDidRestore>(this, OnPropertiesDidRestore);
            Messenger.Default.Register<MapDidUnloadEvent>(this, OnMapDidUnload);
        }

        private void OnDisable()
        {
            Messenger.Default.Unregister(this);
        }

        /// <summary>
        /// Get the state machine for the currently selected locomotive, or null.
        /// </summary>
        public AutopilotStateMachine GetStateMachineForSelected()
        {
            var loco = TrainController.Shared?.SelectedLocomotive;
            if (loco == null) return null;
            _stateMachines.TryGetValue(loco, out var sm);
            return sm;
        }

        public void StartAutopilot()
        {
            var loco = _trainService.GetSelectedLocomotive();
            if (loco == null)
            {
                Loader.Mod.Logger.Log("Autopilot: No locomotive selected.");
                return;
            }

            // Stop existing autopilot for this loco if running
            StopAutopilot(loco);

            var sm = new AutopilotStateMachine(_trainService);
            _stateMachines[loco] = sm;
            sm.Start(loco);
            _coroutines[loco] = StartCoroutine(TickLoop(loco, sm));
        }

        public void StartPickup(string destinationName, bool deliverAfterPickup = false)
        {
            var loco = _trainService.GetSelectedLocomotive();
            if (loco == null)
            {
                Loader.Mod.Logger.Log("Autopilot: No locomotive selected.");
                return;
            }

            StopAutopilot(loco);

            var sm = new AutopilotStateMachine(_trainService);
            sm.DeliverAfterPickup = deliverAfterPickup;
            _stateMachines[loco] = sm;
            sm.Start(loco, AutopilotMode.Pickup, destinationName);
            _coroutines[loco] = StartCoroutine(TickLoop(loco, sm));
        }

        /// <summary>
        /// Start a refuel-only run: enter planning with refuel flag set,
        /// skip delivery/pickup logic, just find facility and refuel.
        /// </summary>
        public void StartRefuel()
        {
            var loco = _trainService.GetSelectedLocomotive();
            if (loco == null)
            {
                Loader.Mod.Logger.Log("Autopilot: No locomotive selected.");
                return;
            }

            // Stop existing autopilot for this loco if running
            StopAutopilot(loco);

            var sm = new AutopilotStateMachine(_trainService);
            sm.RequestRefuel();
            _stateMachines[loco] = sm;
            sm.Start(loco);
            _coroutines[loco] = StartCoroutine(TickLoop(loco, sm));
        }

        /// <summary>
        /// Get destination names for cars reachable from the selected loco.
        /// Used by the UI to populate the pickup destination dropdown.
        /// </summary>
        public List<string> GetReachableDestinations()
        {
            var loco = TrainController.Shared?.SelectedLocomotive;
            if (loco == null) return new List<string>();

            if (_pickupPlanner == null)
                _pickupPlanner = new PickupPlanner(_trainService);

            return _pickupPlanner.GetReachableDestinations(loco);
        }

        public void StopAutopilot()
        {
            var loco = TrainController.Shared?.SelectedLocomotive;
            if (loco != null)
                StopAutopilot(loco);
        }

        private void StopAutopilot(BaseLocomotive loco)
        {
            if (_stateMachines.TryGetValue(loco, out var sm))
                sm.Stop();
            _stateMachines.Remove(loco);

            if (_coroutines.TryGetValue(loco, out var coroutine) && coroutine != null)
            {
                StopCoroutine(coroutine);
                _coroutines.Remove(loco);
            }
        }

        private void StopAll()
        {
            foreach (var kvp in _coroutines)
            {
                if (kvp.Value != null)
                    StopCoroutine(kvp.Value);
            }
            _coroutines.Clear();

            // Don't call sm.Stop() — that would clear persisted state via SetPhase(Idle).
            // The map is unloading, so game objects are being destroyed anyway.
            _stateMachines.Clear();
        }

        private void OnMapDidUnload(MapDidUnloadEvent evt)
        {
            Loader.Mod.Logger.Log("Autopilot: Map unloading, stopping all state machines.");
            StopAll();
        }

        private void ResumeFromSave(BaseLocomotive loco, SavedAutopilotState savedState)
        {
            // If an SM already exists for this loco, clean up without calling Stop()
            // (which would clear persisted state via SetPhase(Idle)).
            if (_stateMachines.Remove(loco) && _coroutines.TryGetValue(loco, out var existing) && existing != null)
            {
                StopCoroutine(existing);
                _coroutines.Remove(loco);
            }

            var sm = new AutopilotStateMachine(_trainService);
            _stateMachines[loco] = sm;
            sm.Resume(loco, savedState);
            _coroutines[loco] = StartCoroutine(TickLoop(loco, sm));

            Loader.Mod.Logger.Log($"Autopilot: Resumed {loco.DisplayName} in {savedState.Mode} mode.");
        }

        private void OnPropertiesDidRestore(PropertiesDidRestore evt)
        {
            if (TrainController.Shared == null)
                return;

            int resumed = 0;
            foreach (var loco in TrainController.Shared.Cars.OfType<BaseLocomotive>())
            {
                var savedState = _trainService.LoadAutopilotState(loco);
                if (savedState != null)
                {
                    ResumeFromSave(loco, savedState);
                    resumed++;
                }
            }

            if (resumed > 0)
                Loader.Mod.Logger.Log($"Autopilot: Resumed {resumed} locomotive(s) from save.");
        }

        public void RetryAutopilot()
        {
            var loco = TrainController.Shared?.SelectedLocomotive;
            if (loco == null) return;

            if (!_stateMachines.TryGetValue(loco, out var sm)) return;

            if (_coroutines.TryGetValue(loco, out var coroutine) && coroutine != null)
                StopCoroutine(coroutine);

            sm.Retry();
            _coroutines[loco] = StartCoroutine(TickLoop(loco, sm));
        }

        private IEnumerator TickLoop(BaseLocomotive loco, AutopilotStateMachine sm)
        {
            while (sm.Phase is not (Idle or Completed or Failed))
            {
                sm.Tick();
                yield return new WaitForSeconds(TickInterval);
            }

            _coroutines.Remove(loco);
        }
    }
}
