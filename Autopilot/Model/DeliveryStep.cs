using System.Collections.Generic;
using Model;
using Model.Ops;

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
        public List<Car> Cars { get; }
        public OpsCarPosition Destination { get; }
        public DirectedPosition DestinationLocation { get; }
        public Car? CoupleTarget { get; }
        public int SpanIndex { get; }
        public StepStatus Status { get; set; }
        public string? ErrorMessage { get; set; }

        public DeliveryStep(List<Car> cars, OpsCarPosition destination, DirectedPosition destinationLocation,
            Car? coupleTarget = null, int spanIndex = 0)
        {
            Cars = cars;
            Destination = destination;
            DestinationLocation = destinationLocation;
            CoupleTarget = coupleTarget;
            SpanIndex = spanIndex;
            Status = StepStatus.Pending;
        }

        public string DestinationName => Destination.DisplayName;

        public string CarNames => string.Join(", ", Cars.ConvertAll(c => c.DisplayName));
    }
}
