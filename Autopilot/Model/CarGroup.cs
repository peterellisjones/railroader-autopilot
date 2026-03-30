using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Autopilot.Services;

namespace Autopilot.Model
{
    public class CarGroup
    {
        private readonly List<ICar> _cars; // ordered tail (outward) to loco (inward)

        public static CarGroup Empty { get; } = new CarGroup(new List<ICar>());

        public IReadOnlyList<ICar> Cars => _cars;
        public ICar? TailCar => _cars.Count > 0 ? _cars[0] : null;
        public ICar? LocomotiveEndCar => _cars.Count > 0 ? _cars[_cars.Count - 1] : null;
        public int Count => _cars.Count;
        public bool IsEmpty => _cars.Count == 0;

        public DirectedPosition? TailOutwardEnd { get; }
        public DirectedPosition? TailInwardEnd { get; }
        public Direction? ConsistDirection { get; }

        public CarGroup(List<ICar> carsFromTailToLoco)
        {
            _cars = carsFromTailToLoco;
            if (_cars.Count > 0)
            {
                ResolveTailEnds(_cars[0], out var outward, out var inward, out var dir);
                TailOutwardEnd = outward;
                TailInwardEnd = inward;
                ConsistDirection = dir;
            }
        }

        /// <summary>
        /// Private constructor for pre-resolved groups (used by Reversed).
        /// </summary>
        private CarGroup(List<ICar> cars, DirectedPosition? tailOutward, DirectedPosition? tailInward, Direction? dir)
        {
            _cars = cars;
            TailOutwardEnd = tailOutward;
            TailInwardEnd = tailInward;
            ConsistDirection = dir;
        }

        public static CarGroup FromSide(List<ICar> side, bool tailAtStart)
        {
            if (side.Count == 0)
                return Empty;

            ICar? tailCar = null;
            var first = side[0];
            var last = side[side.Count - 1];

            bool firstHasFree = first.CoupledTo(Car.LogicalEnd.A) == null || first.CoupledTo(Car.LogicalEnd.B) == null;
            bool lastHasFree = last.CoupledTo(Car.LogicalEnd.A) == null || last.CoupledTo(Car.LogicalEnd.B) == null;

            if (firstHasFree && lastHasFree)
                tailCar = tailAtStart ? first : last;
            else if (firstHasFree)
                tailCar = first;
            else if (lastHasFree)
                tailCar = last;
            else
                tailCar = tailAtStart ? first : last;

            int tailIdx = side.IndexOf(tailCar);
            var ordered = tailIdx == 0 ? new List<ICar>(side) : Enumerable.Reverse(side).ToList();
            return new CarGroup(ordered);
        }

        /// <summary>
        /// Create a reversed view of this group for runaround planning.
        /// The old loco-end car becomes the new tail.
        /// The outward end of the new tail is the end that was facing the loco
        /// (determined by finding which end is NOT coupled to another car in the group).
        /// </summary>
        public CarGroup Reversed()
        {
            if (IsEmpty) return Empty;

            var reversed = new List<ICar>(_cars);
            reversed.Reverse();

            var newTail = reversed[0];

            // Find which end of the new tail faces away from the remaining cars.
            // For the old loco-end car: one end is coupled to the second-to-last car,
            // the other end is coupled to the loco (not in our group) or is free.
            // The end NOT facing a group member is the new outward end.
            if (reversed.Count >= 2)
            {
                var nextCar = reversed[1];
                var coupledAtA = newTail.CoupledTo(Car.LogicalEnd.A);
                bool aFacesNext = coupledAtA != null && coupledAtA.id == nextCar.id;
                // The outward end is the one NOT facing nextCar
                var outwardLogical = aFacesNext ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                var outwardPos = outwardLogical == Car.LogicalEnd.A ? newTail.EndA : newTail.EndB;
                var inwardPos = outwardLogical == Car.LogicalEnd.A ? newTail.EndB : newTail.EndA;

                var dir = ComputeDirection(outwardPos, inwardPos);
                var resolvedOutward = new DirectedPosition(outwardPos.Segment, outwardPos.DistanceFromA, dir);
                var resolvedInward = new DirectedPosition(inwardPos.Segment, inwardPos.DistanceFromA, dir.Opposite());
                return new CarGroup(reversed, resolvedOutward, resolvedInward, dir);
            }
            else
            {
                // Single car — recompute ends fresh
                ResolveTailEnds(newTail, out var outward, out var inward, out var dir);
                // For a single car reversed, swap outward/inward from the original resolution
                // since the loco is now on the other side
                if (TailOutwardEnd.HasValue && TailInwardEnd.HasValue)
                {
                    var swappedDir = ConsistDirection!.Value.Opposite();
                    var swappedOutward = new DirectedPosition(TailInwardEnd.Value.Segment, TailInwardEnd.Value.DistanceFromA, swappedDir);
                    var swappedInward = new DirectedPosition(TailOutwardEnd.Value.Segment, TailOutwardEnd.Value.DistanceFromA, swappedDir.Opposite());
                    return new CarGroup(reversed, swappedOutward, swappedInward, swappedDir);
                }
                return new CarGroup(reversed, inward, outward, dir.Opposite());
            }
        }

        /// <summary>
        /// Resolve the outward/inward ends and direction for a tail car.
        /// The free (uncoupled) end is outward; the coupled end is inward.
        /// Direction is determined by comparing positions on the segment.
        /// </summary>
        private static void ResolveTailEnds(ICar tail,
            out DirectedPosition outward, out DirectedPosition inward, out Direction dir)
        {
            bool aIsFree = tail.CoupledTo(Car.LogicalEnd.A) == null;
            var freeEndPos = aIsFree ? tail.EndA : tail.EndB;
            var coupledEndPos = aIsFree ? tail.EndB : tail.EndA;

            dir = ComputeDirection(freeEndPos, coupledEndPos);
            outward = new DirectedPosition(freeEndPos.Segment, freeEndPos.DistanceFromA, dir);
            inward = new DirectedPosition(coupledEndPos.Segment, coupledEndPos.DistanceFromA, dir.Opposite());
        }

        /// <summary>
        /// Compute the direction from inward toward outward.
        /// Handles same-segment (distance comparison) and cross-segment (shared node walk) cases.
        /// </summary>
        private static Direction ComputeDirection(DirectedPosition outwardPos, DirectedPosition inwardPos)
        {
            if (outwardPos.Segment == inwardPos.Segment && outwardPos.Segment != null)
            {
                return outwardPos.DistanceFromA > inwardPos.DistanceFromA
                    ? Direction.TowardEndB : Direction.TowardEndA;
            }

            if (outwardPos.Segment == null)
                return Direction.TowardEndA; // default

            // Different segments — find connection direction
            var connectNode = TrackWalker.FindSharedNode(outwardPos.Segment, inwardPos.Segment);
            if (connectNode != null)
            {
                var connectEnd = outwardPos.Segment.EndForNode(connectNode);
                // Outward is AWAY from the connection
                return connectEnd == TrackSegment.End.A
                    ? Direction.TowardEndB : Direction.TowardEndA;
            }

            var connectDir = TrackWalker.FindDirectionToward(outwardPos.Segment, inwardPos.Segment);
            if (connectDir.HasValue)
            {
                return connectDir.Value == TrackSegment.End.A
                    ? Direction.TowardEndB : Direction.TowardEndA;
            }

            // Last resort — default
            return outwardPos.DistanceFromA > inwardPos.DistanceFromA
                ? Direction.TowardEndB : Direction.TowardEndA;
        }
    }
}
