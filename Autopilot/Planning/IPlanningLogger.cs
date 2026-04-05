namespace Autopilot.Planning
{
    /// <summary>
    /// Abstract logger for planning code. Enables testing without Loader.Mod.
    /// Game-specific logging methods (LogRoute, LogApproachDetails) stay on
    /// the concrete PlanningLogger — they're only called from game-coupled code.
    /// </summary>
    public interface IPlanningLogger
    {
        void Log(string prefix, string msg);
        void LogDebug(string prefix, string msg);
    }

    /// <summary>No-op logger for unit tests.</summary>
    public class NullPlanningLogger : IPlanningLogger
    {
        public static readonly NullPlanningLogger Instance = new();
        public void Log(string prefix, string msg) { }
        public void LogDebug(string prefix, string msg) { }
    }
}
