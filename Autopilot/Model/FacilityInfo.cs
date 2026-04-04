using Track;

namespace Autopilot.Model
{
    /// <summary>
    /// A discovered fuel/water/coal facility on the track.
    /// </summary>
    public class FacilityInfo
    {
        /// <summary>The fuel type this facility provides (e.g. "water", "coal", "diesel-fuel").</summary>
        public string FuelType { get; }

        /// <summary>Track position of the loader.</summary>
        public DirectedPosition Location { get; }

        /// <summary>
        /// Stable registered ID of the CarLoadTargetLoader's KeyValueObject.
        /// Used to re-find the loader and its matching CarLoaderSequencer at activation time.
        /// Unlike world position, this ID is not affected by floating-origin shifts.
        /// </summary>
        public string LoaderRegisteredId { get; }

        /// <summary>Graph distance from the locomotive to this facility. Set during selection.</summary>
        public float Distance { get; set; }

        /// <summary>
        /// Industry identifier, if this loader belongs to an industry.
        /// Null for standalone water towers.
        /// </summary>
        public string? IndustryId { get; }

        public FacilityInfo(string fuelType, DirectedPosition location, string loaderRegisteredId, string? industryId)
        {
            FuelType = fuelType;
            Location = location;
            LoaderRegisteredId = loaderRegisteredId;
            IndustryId = industryId;
        }
    }
}
