using System.Collections.Generic;
using Autopilot.Model;

namespace Autopilot.Services
{
    /// <summary>
    /// Abstraction over game locomotive/car/track queries for the planning layer.
    /// Scoped to one locomotive — no BaseLocomotive parameter on methods.
    /// Uses game-type-free types: GraphPosition, ICar, string IDs.
    /// </summary>
    public interface ITrainService
    {
        // --- Consist ---
        IReadOnlyList<ICar> GetCoupled();
        float GetTrainLength();
        ICar GetLocoCar();
        string LocoId { get; }

        // --- Positions ---
        GraphPosition GetLocoFront();
        GraphPosition GetLocoRear();
        GraphPosition GetCarEndA(ICar car);
        GraphPosition GetCarEndB(ICar car);
        GraphPosition GetCarFront(ICar car);
        GraphPosition GetCarRear(ICar car);

        // --- Car data ---
        bool HasWaybill(ICar car);
        bool IsWaybillCompleted(ICar car);
        string? GetDestinationTrackId(ICar car);
        string? GetDestinationName(ICar car);

        // --- Destination queries (wraps DestinationSelector) ---
        IReadOnlyList<DestinationCandidate> GetDestinationCandidates(ICar car);
        float GetAvailableSpace(ICar car);

        // --- Loop/runaround queries (wraps LoopValidator) ---
        LoopStatus GetLoopStatus();
        (GraphPosition? location, string? loopKey) GetRepositionLocation(
            IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys,
            IReadOnlyList<GraphPosition>? deliveryDestinations);

        // --- Cache ---
        void ClearPlanCaches();
    }
}
