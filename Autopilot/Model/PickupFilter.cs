using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace Autopilot.Model
{
    public enum FilterMode
    {
        Any,
        Area,
        Industry,
        Destination,
        Switchlist,
    }

    public class FilterAxis
    {
        public FilterMode Mode { get; }
        public HashSet<string> CheckedItems { get; }

        public FilterAxis(FilterMode mode, HashSet<string> checkedItems)
        {
            Mode = mode;
            CheckedItems = checkedItems;
        }

        /// <summary>Returns a new FilterAxis that accepts everything.</summary>
        public static FilterAxis Any => new FilterAxis(FilterMode.Any, new HashSet<string>());

        /// <summary>
        /// Human-readable label for this axis used in DisplaySummary.
        ///   Switchlist           → "Switchlist"
        ///   Any                  → "Any"
        ///   ≤2 checked items     → comma-joined names
        ///   >2 checked items     → "N ModeName(s)"
        /// </summary>
        internal string Label
        {
            get
            {
                if (Mode == FilterMode.Switchlist)
                    return "Switchlist";

                if (Mode == FilterMode.Any)
                    return "Any";

                // Named-item modes (Area, Industry, Destination)
                if (CheckedItems.Count == 0)
                    return Mode.ToString();
                if (CheckedItems.Count <= 2)
                    return string.Join(", ", CheckedItems);

                // >2 items: "N Areas" / "N Industries" / "N Destinations"
                string plural = Mode switch
                {
                    FilterMode.Area        => "Areas",
                    FilterMode.Industry    => "Industries",
                    FilterMode.Destination => "Destinations",
                    _                      => $"{Mode}s",
                };
                return $"{CheckedItems.Count} {plural}";
            }
        }
    }

    public class PickupFilter
    {
        public FilterAxis From { get; set; }
        public FilterAxis To { get; set; }
        public float MaxDistance { get; set; }
        public bool AutoAddToSwitchlist { get; set; }

        public PickupFilter(FilterAxis from, FilterAxis to, float maxDistance, bool autoAddToSwitchlist)
        {
            From = from;
            To = to;
            MaxDistance = maxDistance;
            AutoAddToSwitchlist = autoAddToSwitchlist;
        }

        /// <summary>Returns the default filter: accept all cars, no distance limit.</summary>
        public static PickupFilter Default =>
            new PickupFilter(FilterAxis.Any, FilterAxis.Any, float.MaxValue, autoAddToSwitchlist: false);

        // ── Display ──────────────────────────────────────────────────────────────

        public string DisplaySummary
        {
            get
            {
                if (From.Mode == FilterMode.Any && To.Mode == FilterMode.Any)
                    return "All cars";

                return $"From: {From.Label} \u2192 To: {To.Label}";
            }
        }

        // ── Serialization ────────────────────────────────────────────────────────

        public string Serialize()
        {
            var obj = new JObject
            {
                ["from"] = SerializeAxis(From),
                ["to"]   = SerializeAxis(To),
                ["maxDistance"]       = MaxDistance,
                ["autoAddToSwitchlist"] = AutoAddToSwitchlist,
            };
            return obj.ToString(Formatting.None);
        }

        private static JObject SerializeAxis(FilterAxis axis)
        {
            return new JObject
            {
                ["mode"]         = axis.Mode.ToString(),
                ["checkedItems"] = new JArray(axis.CheckedItems.ToArray()),
            };
        }

        /// <summary>
        /// Deserializes a PickupFilter from JSON.
        /// Returns <see cref="Default"/> for null, empty, or invalid input.
        /// </summary>
        public static PickupFilter Deserialize(string? json)
        {
            if (string.IsNullOrEmpty(json))
                return Default;

            try
            {
                var obj = JObject.Parse(json);
                var from = DeserializeAxis(obj["from"] as JObject);
                var to   = DeserializeAxis(obj["to"]   as JObject);
                var maxDistance = obj["maxDistance"]?.Value<float>() ?? float.MaxValue;
                var autoAdd     = obj["autoAddToSwitchlist"]?.Value<bool>() ?? false;
                return new PickupFilter(from, to, maxDistance, autoAdd);
            }
            catch
            {
                return Default;
            }
        }

        private static FilterAxis DeserializeAxis(JObject? obj)
        {
            if (obj == null)
                return FilterAxis.Any;

            var modeStr = obj["mode"]?.Value<string>() ?? "Any";
            if (!System.Enum.TryParse(modeStr, out FilterMode mode))
                mode = FilterMode.Any;

            var items = new HashSet<string>();
            if (obj["checkedItems"] is JArray arr)
            {
                foreach (var token in arr)
                {
                    var s = token.Value<string>();
                    if (s != null)
                        items.Add(s);
                }
            }

            return new FilterAxis(mode, items);
        }
    }
}
