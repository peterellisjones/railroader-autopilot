using System.Collections.Generic;
using NUnit.Framework;
using Autopilot.Model;
using Autopilot.Planning;
using Model.Ops;
using Track;

namespace Autopilot.Tests
{
    [TestFixture]
    public class PickupFilterTests
    {
        // ── Round-trip serialization ──────────────────────────────────────────────

        [Test]
        public void RoundTrip_defaults()
        {
            var filter = PickupFilter.Default;
            var json = filter.Serialize();
            var restored = PickupFilter.Deserialize(json);

            Assert.AreEqual(FilterMode.Any, restored.From.Mode);
            Assert.AreEqual(0, restored.From.CheckedItems.Count);
            Assert.AreEqual(FilterMode.Any, restored.To.Mode);
            Assert.AreEqual(0, restored.To.CheckedItems.Count);
            Assert.AreEqual(float.MaxValue, restored.MaxDistance);
            Assert.IsFalse(restored.AutoAddToSwitchlist);
        }

        [Test]
        public void RoundTrip_with_checked_items()
        {
            var from = new FilterAxis(FilterMode.Area, new HashSet<string> { "Sylva", "Balsam" });
            var to = new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" });
            var filter = new PickupFilter(from, to, 500f, autoAddToSwitchlist: true);

            var restored = PickupFilter.Deserialize(filter.Serialize());

            Assert.AreEqual(FilterMode.Area, restored.From.Mode);
            Assert.IsTrue(restored.From.CheckedItems.Contains("Sylva"));
            Assert.IsTrue(restored.From.CheckedItems.Contains("Balsam"));
            Assert.AreEqual(2, restored.From.CheckedItems.Count);

            Assert.AreEqual(FilterMode.Destination, restored.To.Mode);
            Assert.IsTrue(restored.To.CheckedItems.Contains("Mill S1"));
            Assert.AreEqual(1, restored.To.CheckedItems.Count);

            Assert.AreEqual(500f, restored.MaxDistance, 0.001f);
            Assert.IsTrue(restored.AutoAddToSwitchlist);
        }

        [Test]
        public void RoundTrip_switchlist_mode_no_checked_items()
        {
            var from = new FilterAxis(FilterMode.Switchlist, new HashSet<string>());
            var filter = new PickupFilter(from, FilterAxis.Any, float.MaxValue, autoAddToSwitchlist: false);

            var restored = PickupFilter.Deserialize(filter.Serialize());

            Assert.AreEqual(FilterMode.Switchlist, restored.From.Mode);
            Assert.AreEqual(0, restored.From.CheckedItems.Count);
            Assert.AreEqual(FilterMode.Any, restored.To.Mode);
        }

        // ── Null / empty input ────────────────────────────────────────────────────

        [Test]
        public void Deserialize_null_or_empty_returns_default()
        {
            var fromNull = PickupFilter.Deserialize(null);
            var fromEmpty = PickupFilter.Deserialize("");

            Assert.AreEqual(FilterMode.Any, fromNull.From.Mode);
            Assert.AreEqual(FilterMode.Any, fromNull.To.Mode);
            Assert.AreEqual(float.MaxValue, fromNull.MaxDistance);
            Assert.IsFalse(fromNull.AutoAddToSwitchlist);

            Assert.AreEqual(FilterMode.Any, fromEmpty.From.Mode);
            Assert.AreEqual(FilterMode.Any, fromEmpty.To.Mode);
        }

        // ── DisplaySummary ────────────────────────────────────────────────────────

        [Test]
        public void DisplaySummary_any_any()
        {
            Assert.AreEqual("All cars", PickupFilter.Default.DisplaySummary);
        }

        [Test]
        public void DisplaySummary_area_destination()
        {
            var from = new FilterAxis(FilterMode.Area, new HashSet<string> { "Sylva" });
            var to = new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" });
            var filter = new PickupFilter(from, to, float.MaxValue, false);

            Assert.AreEqual("From: Sylva → To: Mill S1", filter.DisplaySummary);
        }

        [Test]
        public void DisplaySummary_switchlist_from()
        {
            var from = new FilterAxis(FilterMode.Switchlist, new HashSet<string>());
            var filter = new PickupFilter(from, FilterAxis.Any, float.MaxValue, false);

            // Switchlist mode → label is "Switchlist"; To is Any so omitted from label
            // Full summary: "From: Switchlist → To: Any" or similar
            // Per spec: when one axis is non-Any, show "From: X → To: Y"
            // Any axis shows its mode name: "Any"; Switchlist shows "Switchlist"
            Assert.AreEqual("From: Switchlist → To: Any", filter.DisplaySummary);
        }

        [Test]
        public void DisplaySummary_many_items_shows_count_and_mode()
        {
            var from = new FilterAxis(FilterMode.Industry, new HashSet<string> { "Ind1", "Ind2", "Ind3" });
            var filter = new PickupFilter(from, FilterAxis.Any, float.MaxValue, false);

            // >2 items → "3 Industries" (pluralised mode name)
            Assert.AreEqual("From: 3 Industries → To: Any", filter.DisplaySummary);
        }

        [Test]
        public void DisplaySummary_two_items_shows_joined_names()
        {
            var from = new FilterAxis(FilterMode.Area, new HashSet<string> { "Sylva", "Balsam" });
            var filter = new PickupFilter(from, FilterAxis.Any, float.MaxValue, false);

            var summary = filter.DisplaySummary;
            // ≤2 items → joined names; order may vary since HashSet is unordered
            Assert.IsTrue(
                summary == "From: Sylva, Balsam → To: Any" ||
                summary == "From: Balsam, Sylva → To: Any",
                $"Unexpected summary: {summary}");
        }

        [Test]
        public void DisplaySummary_no_checked_items_shows_mode_name()
        {
            var from = new FilterAxis(FilterMode.Area, new HashSet<string>());
            var filter = new PickupFilter(from, FilterAxis.Any, float.MaxValue, false);

            Assert.AreEqual("From: Area → To: Any", filter.DisplaySummary);
        }

        // ── MatchesFilter ────────────────────────────────────────────────────────

        [Test]
        public void MatchesCar_any_any_matches_car_with_waybill()
        {
            var car = new MockCar
            {
                id = "c1",
                DisplayName = "Box Car",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, false, null, 0)
            };
            var filter = PickupFilter.Default;
            Assert.IsTrue(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        [Test]
        public void MatchesCar_completed_waybill_does_not_match()
        {
            var car = new MockCar
            {
                id = "c1",
                DisplayName = "Box Car",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, true, null, 0)
            };
            var filter = PickupFilter.Default;
            Assert.IsFalse(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        [Test]
        public void MatchesCar_no_waybill_does_not_match()
        {
            var car = new MockCar { id = "c1", DisplayName = "Box Car", Waybill = null };
            var filter = PickupFilter.Default;
            Assert.IsFalse(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        [Test]
        public void MatchesCar_to_destination_matches()
        {
            var car = new MockCar
            {
                id = "c1",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, false, null, 0)
            };
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" }),
                float.MaxValue, false);
            Assert.IsTrue(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        [Test]
        public void MatchesCar_to_destination_no_match()
        {
            var car = new MockCar
            {
                id = "c1",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, false, null, 0)
            };
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Lumber S2" }),
                float.MaxValue, false);
            Assert.IsFalse(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        [Test]
        public void MatchesCar_switchlist_matches_when_on_list()
        {
            var car = new MockCar
            {
                id = "c1",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, false, null, 0)
            };
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                FilterAxis.Any,
                float.MaxValue, false);
            Assert.IsTrue(PickupPlanner.MatchesFilter(car, filter, new HashSet<string> { "c1" }));
        }

        [Test]
        public void MatchesCar_switchlist_no_match_when_not_on_list()
        {
            var car = new MockCar
            {
                id = "c1",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, false, null, 0)
            };
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                FilterAxis.Any,
                float.MaxValue, false);
            Assert.IsFalse(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()));
        }

        private static OpsCarPosition TestOpsCarPosition(string displayName, string identifier)
        {
            return new OpsCarPosition(displayName, identifier, System.Array.Empty<TrackSpan>());
        }
    }
}
