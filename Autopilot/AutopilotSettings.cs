using UnityEngine;
using UnityModManagerNet;

namespace Autopilot
{
    public class AutopilotSettings : UnityModManager.ModSettings, IDrawable
    {
        [Header("Keybindings")]
        [Draw("Toggle Autopilot window")] public KeyBinding toggleWindowKey = new KeyBinding() { modifiers = 2, keyCode = KeyCode.N };

        [Header("Debug")]
        [Draw("Verbose logging")] public bool verboseLogging = false;


        public override void Save(UnityModManager.ModEntry modEntry)
        {
            Save(this, modEntry);
        }

        public void OnChange()
        {
        }
    }
}
