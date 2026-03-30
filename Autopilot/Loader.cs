using System;
using HarmonyLib;
using UnityEngine;
using UnityModManagerNet;
using Autopilot.UI;

namespace Autopilot
{
    public static class Loader
    {
        public static UnityModManager.ModEntry Mod { get; private set; }
        public static AutopilotSettings Settings { get; private set; }
        private static Harmony _harmony;
        private static GameObject _controllerGo;
        private static AutopilotWindow _window;

        public static bool Load(UnityModManager.ModEntry modEntry)
        {
            Mod = modEntry;
            Settings = UnityModManager.ModSettings.Load<AutopilotSettings>(modEntry);

            try
            {
                _harmony = new Harmony(modEntry.Info.Id);
                _harmony.PatchAll(typeof(Loader).Assembly);
                Mod.Logger.Log("Autopilot: Harmony patches applied.");
            }
            catch (Exception ex)
            {
                Mod.Logger.LogException("Autopilot: Failed to apply Harmony patches.", ex);
                return false;
            }

            _controllerGo = new GameObject("AutopilotController");
            _controllerGo.AddComponent<AutopilotController>();
            UnityEngine.Object.DontDestroyOnLoad(_controllerGo);

            _window = new AutopilotWindow();

            modEntry.OnUpdate = OnUpdate;
            modEntry.OnGUI = OnGUI;
            modEntry.OnSaveGUI = OnSaveGUI;

            Mod.Logger.Log("Autopilot mod loaded.");
            return true;
        }

        private static void OnUpdate(UnityModManager.ModEntry modEntry, float deltaTime)
        {
            if (Settings.toggleWindowKey.Down())
            {
                _window.Toggle();
            }
        }

        private static void OnGUI(UnityModManager.ModEntry modEntry)
        {
            Settings.Draw(modEntry);
        }

        private static void OnSaveGUI(UnityModManager.ModEntry modEntry)
        {
            Settings.Save(modEntry);
        }
    }
}
