using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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

        public void StartPickup(string destinationName)
        {
            var loco = _trainService.GetSelectedLocomotive();
            if (loco == null)
            {
                Loader.Mod.Logger.Log("Autopilot: No locomotive selected.");
                return;
            }

            StopAutopilot(loco);

            var sm = new AutopilotStateMachine(_trainService);
            _stateMachines[loco] = sm;
            sm.Start(loco, AutopilotMode.Pickup, destinationName);
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
