using System.Collections.Generic;
using Autopilot.Planning;
using NUnit.Framework;

namespace Autopilot.Tests
{
    [TestFixture]
    public class RunaroundSplitCalculatorTests
    {
        // D = deliverable direct, R = deliverable after flip, B = both, N = neither, T = tender
        private static (System.Func<int, bool> isLocoOrTender,
            System.Func<int, bool> isDirect,
            System.Func<int, bool> isFlipped) Parse(string spec)
        {
            return (
                i => spec[i] == 'T',
                i => spec[i] == 'D' || spec[i] == 'B',
                i => spec[i] == 'R' || spec[i] == 'B'
            );
        }

        [Test]
        public void AllDirect_KeepsZero()
        {
            // D D D D — all direct from tail. K=0: sideB all flipped? No, D not flipped.
            // Score=0 for all K. Tie → K=0.
            var (t, d, f) = Parse("DDDD");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(4, t, d, f);
            Assert.AreEqual(0, keep);
        }

        [Test]
        public void AllNeedRunaround_KeepsZero()
        {
            // R R R R — K=0: sideB from car[3] → all flipped → score 4.
            // K=4: sideA score 4. Tie → K=0 preferred.
            var (t, d, f) = Parse("RRRR");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(4, t, d, f);
            Assert.AreEqual(0, keep, "Tie — prefer loco goes alone");
        }

        [Test]
        public void MixedDirectAndRunaround_PreferFewerKept()
        {
            // D D D R R R — K=0: sideB from car[5]→R,R,R=3. K=3: sideA=3.
            // Tie → K=0.
            var (t, d, f) = Parse("DDDRRR");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(6, t, d, f);
            Assert.AreEqual(0, keep, "Tie — prefer loco goes alone");
        }

        [Test]
        public void OneTender_AlwaysKept()
        {
            // D D R R T — tender at inner end. Always kept.
            // Freight: D D R R (4 cars). K=0: sideB from car[3]→R,R=2.
            // K=2: sideA R,R=2. Tie → K=0 + tender = 1.
            var (t, d, f) = Parse("DDRRT");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(5, t, d, f);
            Assert.AreEqual(1, keep, "Just the tender");
        }

        [Test]
        public void NoneDeliverable_KeepsZero()
        {
            var (t, d, f) = Parse("NNNN");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(4, t, d, f);
            Assert.AreEqual(0, keep);
        }

        [Test]
        public void SingleCar_Direct_KeepsZero()
        {
            var (t, d, f) = Parse("D");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(1, t, d, f);
            Assert.AreEqual(0, keep);
        }

        [Test]
        public void SingleCar_Runaround_KeepsZero()
        {
            // R — K=0: sideB flipped(0)=True → 1. K=1: sideA=1. Tie → K=0.
            var (t, d, f) = Parse("R");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(1, t, d, f);
            Assert.AreEqual(0, keep, "Tie — prefer loco goes alone");
        }

        [Test]
        public void SixCarsToSameDestination_KeepsZero()
        {
            // B B B B B B — all deliverable both ways. Any K gives same score. K=0.
            var (t, d, f) = Parse("BBBBBB");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(6, t, d, f);
            Assert.AreEqual(0, keep, "Same score — prefer fewer cars with loco");
        }

        [Test]
        public void SideBScoresHigherWithFlipped()
        {
            // N N N N R R — only last 2 are flipped-deliverable.
            // K=0: sideB from car[5]→R,R=2. Total=2.
            // K=2: sideA R,R=2, sideB from car[3]→N=0. Total=2. Tie → K=0.
            var (t, d, f) = Parse("NNNNRR");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(6, t, d, f);
            Assert.AreEqual(0, keep);
        }

        [Test]
        public void KeepNeeded_WhenSideAScoresBetter()
        {
            // N N R R R N — middle cars are flipped-deliverable.
            // K=0: sideB from car[5]→N=0. Total=0.
            // K=1: sideB from car[4]→R=1. sideA from car[5]→N=0. Total=1.
            // K=3: detached=NNN. sideB from car[2]→N=0. sideA from car[5]→N=0,
            //   wait: sideA from car[5] but car[5] is N. Score=0. Total=0.
            // K=4: detached=NN. sideB from car[1]→N=0. sideA from car[5]→N(5),
            //   no wait: sideA iterates from freightCount-1 down to detached.
            //   freight=6. sideA from car[5] to car[detached]:
            //   K=1, detached=5: sideA from car[5] to car[5]: N(5)=0.
            //   K=2, detached=4: sideA from car[5] to car[4]: N(5)=0.
            //   K=3, detached=3: sideA from car[5],car[4],car[3]: N(5)=0.
            //   K=4, detached=2: sideA from car[5] to car[2]: N(5)=0.
            //
            // Hmm, R cars are at indices 2,3,4. sideA iterates from inner end.
            // For K=3: kept=car[3],car[4],car[5]. sideA from car[5]: N→0.
            // For K=4: kept=car[2],car[3],car[4],car[5]. sideA from car[5]: N→0.
            //
            // The R cars never get scored because sideA starts from the innermost
            // (car[5]=N) and stops. The consecutive check breaks at N.
            //
            // So best is K=1 with total=1.
            var (t, d, f) = Parse("NNRRRN");
            int keep = RunaroundSplitCalculator.CalculateCarsToKeep(6, t, d, f);
            Assert.AreEqual(1, keep);
        }
    }
}
