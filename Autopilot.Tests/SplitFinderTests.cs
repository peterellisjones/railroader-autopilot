using System.Collections.Generic;
using Autopilot.Planning;
using NUnit.Framework;

namespace Autopilot.Tests
{
    [TestFixture]
    public class SplitFinderTests
    {
        [Test]
        public void IsRouteSafe_NoBlockedSwitches_ReturnsTrue()
        {
            var blocked = new HashSet<string>();
            var steps = new List<Track.Search.RouteSearch.Step>();
            Assert.IsTrue(SplitFinder.IsRouteSafe(steps, blocked));
        }

        [Test]
        public void IsRouteSafe_EmptyRoute_ReturnsTrue()
        {
            var blocked = new HashSet<string> { "switchA", "switchB" };
            var steps = new List<Track.Search.RouteSearch.Step>();
            Assert.IsTrue(SplitFinder.IsRouteSafe(steps, blocked));
        }
    }
}
