namespace Autopilot.Model
{
    /// <summary>
    /// A candidate location for delivering cars. Returned by ITrainService.GetDestinationCandidates.
    /// Wraps the result of DestinationSelector without exposing game types.
    /// </summary>
    public readonly record struct DestinationCandidate(
        GraphPosition Location,
        ICar? CoupleTarget,
        float AvailableSpace,
        int SpanIndex,
        GraphPosition ApproachTarget,
        string DestinationTrackId,
        string DestinationName);
}
