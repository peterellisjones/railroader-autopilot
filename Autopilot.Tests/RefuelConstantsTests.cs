using NUnit.Framework;

namespace Autopilot.Tests
{
    [TestFixture]
    public class RefuelConstantsTests
    {
        [Test]
        public void RefuelApproachSpeed_is_5mph()
        {
            Assert.AreEqual(5, AutopilotConstants.RefuelApproachSpeedMph);
        }

        [Test]
        public void OpportunisticMaxPercent_is_90()
        {
            Assert.AreEqual(90f, AutopilotConstants.OpportunisticMaxPercent);
        }

        [Test]
        public void NearbyFacilityDistance_is_100m()
        {
            Assert.AreEqual(100f, AutopilotConstants.NearbyFacilityDistanceMeters);
        }

        [Test]
        public void FullThreshold_is_100_percent()
        {
            Assert.AreEqual(100f, AutopilotConstants.FullThresholdPercent);
        }

        [Test]
        public void DefaultMidRunThreshold_is_20()
        {
            var settings = new AutopilotSettings();
            Assert.AreEqual(20, settings.midRunRefuelThreshold);
        }

        [Test]
        public void DefaultCompletionThreshold_is_50()
        {
            var settings = new AutopilotSettings();
            Assert.AreEqual(50, settings.completionRefuelThreshold);
        }
    }
}
