using Track;
using UnityEngine;

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

        /// <summary>World position of the loader (for matching to CarLoaderSequencer).</summary>
        public Vector3 WorldPosition { get; }

        /// <summary>Graph distance from the locomotive to this facility. Set during selection.</summary>
        public float Distance { get; set; }

        /// <summary>
        /// Industry identifier, if this loader belongs to an industry.
        /// Null for standalone water towers.
        /// </summary>
        public string? IndustryId { get; }

        public FacilityInfo(string fuelType, DirectedPosition location, Vector3 worldPosition, string? industryId)
        {
            FuelType = fuelType;
            Location = location;
            WorldPosition = worldPosition;
            IndustryId = industryId;
        }
    }
}
