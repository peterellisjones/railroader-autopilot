using System.Collections.Generic;
using System.Linq;

namespace Autopilot.Model
{
    public enum StepStatus
    {
        Pending,
        InProgress,
        Done,
        Error
    }

    public class DeliveryStep
    {
        public IReadOnlyList<ICar> Cars { get; }
        public string DestinationTrackId { get; }
        public string DestinationName { get; }
        public GraphPosition DestinationLocation { get; }
        public ICar? CoupleTarget { get; }
        public int SpanIndex { get; }
        public StepStatus Status { get; set; }
        public string? ErrorMessage { get; set; }

        public DeliveryStep(IReadOnlyList<ICar> cars, string destinationTrackId,
            string destinationName, GraphPosition destinationLocation,
            ICar? coupleTarget = null, int spanIndex = 0)
        {
            Cars = cars;
            DestinationTrackId = destinationTrackId;
            DestinationName = destinationName;
            DestinationLocation = destinationLocation;
            CoupleTarget = coupleTarget;
            SpanIndex = spanIndex;
            Status = StepStatus.Pending;
        }

        public string CarNames => string.Join(", ", Cars.Select(c => c.DisplayName));
    }
}
