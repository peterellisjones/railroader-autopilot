using Model;

namespace Autopilot.Model
{
    /// <summary>
    /// A typed coupling waypoint — makes it impossible to couple to the wrong end.
    /// Position already encodes which end and approach direction.
    /// Produced by CoupleLocationCalculator which evaluates both ends.
    /// </summary>
    public readonly record struct CoupleTarget(
        DirectedPosition Position,
        Car TargetCar
    );
}
