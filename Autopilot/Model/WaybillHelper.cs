using Model.Ops;
using Track;

namespace Autopilot.Model
{
    public static class WaybillHelper
    {
        /// <summary>
        /// Returns true if the car has a waybill and still needs to be delivered.
        /// A car is "pending delivery" if:
        ///   - It has a non-completed waybill, OR
        ///   - It has a completed waybill but has been moved away from its destination
        /// </summary>
        public static bool IsPendingDelivery(ICar car)
        {
            var waybill = car.Waybill;
            if (waybill == null) return false;

            if (!waybill.Value.Completed) return true;

            // Completed waybill — check if the car is still at the destination.
            // If it's been displaced, it needs to be delivered again.
            return !IsAtDestination(car, waybill.Value.Destination);
        }

        /// <summary>
        /// Returns true if either end of the car is within one of the destination's track spans.
        /// </summary>
        public static bool IsAtDestination(ICar car, OpsCarPosition destination)
        {
            // Guard against null segments (can happen in tests or if car isn't on track)
            if (car.EndA.Segment == null || car.EndB.Segment == null)
                return false;

            var locA = car.EndA.ToLocation();
            var locB = car.EndB.ToLocation();

            foreach (var span in destination.Spans)
            {
                if (span.Contains(locA) || span.Contains(locB))
                    return true;
            }
            return false;
        }
    }
}
