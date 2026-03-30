using System.Collections.Generic;
using Track;
using Autopilot.Model;
using Autopilot.Patches;

namespace Autopilot.Services
{
    public static class TrackWalker
    {
        public static (TrackNode switchNode, float distance)? WalkToSwitch(
            TrackSegment segment, TrackSegment.End walkEnd, int maxSegments = 30)
        {
            var graph = Graph.Shared;
            float distanceWalked = 0f;

            for (int i = 0; i < maxSegments; i++)
            {
                var node = segment.NodeForEnd(walkEnd);
                if (node == null) break;

                if (graph.IsSwitch(node))
                    return (node, distanceWalked);

                GraphPatches.SegmentsReachableFrom(graph, segment, walkEnd,
                    out TrackSegment next, out _);
                if (next == null) break;

                distanceWalked += next.GetLength();
                var otherNode = next.GetOtherNode(node);
                if (otherNode == null) break;
                walkEnd = next.EndForNode(otherNode);
                segment = next;
            }

            return null;
        }

        public static DirectedPosition WalkToBack(DirectedPosition front, float trainLength)
        {
            var graph = Graph.Shared;
            var frontLoc = front.ToLocation();
            var backLoc = graph.LocationByMoving(frontLoc.Flipped(), trainLength).Flipped();
            return DirectedPosition.FromLocation(backLoc);
        }

        public static float ClearanceDistance(float distanceWalked, TrackNode switchNode,
            TrackSegment approachSegment)
        {
            var graph = Graph.Shared;
            graph.DecodeSwitchAt(switchNode, out TrackSegment switchEnter, out _, out _);
            float fouling = graph.CalculateFoulingDistance(switchNode);
            bool facingEntrance = (switchEnter == approachSegment);

            float clearance = facingEntrance ? fouling : 2f;
            return distanceWalked + clearance;
        }

        /// <summary>
        /// Does a route start by going in the outward direction from the tail?
        ///
        /// Outward is determined by comparing the outward and inward POSITIONS
        /// on the segment (using canonical DistanceFromA). If outward.DistanceFromA
        /// &lt; inward.DistanceFromA, outward is toward End.A (lower distances).
        /// If outward.DistanceFromA &gt; inward.DistanceFromA, outward is toward End.B.
        /// </summary>
        public static bool RouteGoesOutward(DirectedPosition outward, DirectedPosition inward,
            List<TrackSegment> routeSegments)
        {
            if (outward.Segment == null || routeSegments == null || routeSegments.Count < 2)
                return true;

            // Determine outward direction from position comparison
            // (only works when both locations are on the same segment)
            TrackSegment.End outwardEnd;
            if (outward.Segment == inward.Segment)
            {
                // Already canonical — DistanceFromA is directly comparable
                outwardEnd = outward.DistanceFromA < inward.DistanceFromA
                    ? TrackSegment.End.A : TrackSegment.End.B;
            }
            else
            {
                // Different segments — find which end of the outward segment
                // connects to the inward segment. The outward direction is the
                // OPPOSITE end (away from the inward segment).
                var connectNode = FindSharedNode(outward.Segment, inward.Segment);
                if (connectNode != null)
                {
                    var connectEnd = outward.Segment.EndForNode(connectNode);
                    outwardEnd = connectEnd == TrackSegment.End.A
                        ? TrackSegment.End.B : TrackSegment.End.A;
                }
                else
                {
                    // No shared node — walk the graph to find the connection
                    var connectDir = FindDirectionToward(outward.Segment, inward.Segment);
                    if (connectDir.HasValue)
                    {
                        outwardEnd = connectDir.Value == TrackSegment.End.A
                            ? TrackSegment.End.B : TrackSegment.End.A;
                    }
                    else
                    {
                        // Can't determine direction — assume outward (optimistic default)
                        return true;
                    }
                }
            }

            // Find which end the route exits from
            TrackSegment secondSeg = null;
            foreach (var seg in routeSegments)
            {
                if (seg != outward.Segment)
                {
                    secondSeg = seg;
                    break;
                }
            }

            if (secondSeg == null)
                return true;

            var sharedNode = FindSharedNode(outward.Segment, secondSeg);
            if (sharedNode == null)
                return true;

            var exitEnd = outward.Segment.EndForNode(sharedNode);
            return exitEnd == outwardEnd;
        }

        /// <summary>
        /// Find the best position on a branch to park a train without fouling switches.
        /// Returns the waypoint distance from the branch start, or null if no gap fits.
        ///
        /// The branch has fouling zones at each switch (start, end, and intermediates).
        /// The train should park in the largest gap between fouling zones.
        /// </summary>
        /// <param name="branchLength">Total branch length.</param>
        /// <param name="foulingAtStart">Fouling zone at the start switch.</param>
        /// <param name="foulingAtEnd">Fouling zone at the end switch.</param>
        /// <param name="intermediateSwitchPositions">
        /// List of (distance from branch start, fouling radius) for each intermediate switch.
        /// </param>
        /// <param name="trainLength">Length of the train.</param>
        /// <returns>
        /// Waypoint distance from the branch start (loco position), or null if no gap fits.
        /// </returns>
        public static float? FindBestGapPosition(
            float branchLength,
            float foulingAtStart,
            float foulingAtEnd,
            List<(float position, float fouling)> intermediateSwitchPositions,
            float trainLength)
        {
            // Build list of fouling zones (no-go areas)
            var zones = new List<(float start, float end)>();
            zones.Add((0f, foulingAtStart));

            if (intermediateSwitchPositions != null)
            {
                foreach (var (pos, fouling) in intermediateSwitchPositions)
                    zones.Add((pos - fouling, pos + fouling));
            }

            zones.Add((branchLength - foulingAtEnd, branchLength));
            zones.Sort((a, b) => a.start.CompareTo(b.start));

            // Find the FIRST gap (closest to start) that fits the train.
            // Shortest distance from the entry switch = less movement.
            float bestWaypointDist = -1f;

            for (int i = 0; i < zones.Count - 1; i++)
            {
                float gapStart = zones[i].end;
                float gapEnd = zones[i + 1].start;
                float gapSize = gapEnd - gapStart;

                if (gapSize >= trainLength)
                {
                    // Loco at the far end of the gap, tail at the near end
                    bestWaypointDist = gapStart + trainLength;
                    break; // first fitting gap = closest to entry
                }
            }

            return bestWaypointDist >= 0f ? bestWaypointDist : (float?)null;
        }

        /// <summary>
        /// Walk from a segment in both directions looking for a target segment.
        /// Returns the End of fromSeg that connects toward targetSeg, or null
        /// if the connection cannot be found within maxHops.
        /// Used to determine direction without relying on Location.end.
        /// </summary>
        public static TrackSegment.End? FindDirectionToward(
            TrackSegment fromSeg, TrackSegment targetSeg, int maxHops = 5)
        {
            if (fromSeg == null || targetSeg == null || fromSeg == targetSeg)
                return null;

            if (WalkReachesTarget(fromSeg, TrackSegment.End.A, targetSeg, maxHops))
                return TrackSegment.End.A;

            if (WalkReachesTarget(fromSeg, TrackSegment.End.B, targetSeg, maxHops))
                return TrackSegment.End.B;

            return null;
        }

        private static bool WalkReachesTarget(
            TrackSegment startSeg, TrackSegment.End walkEnd,
            TrackSegment target, int maxHops)
        {
            var graph = Graph.Shared;
            var seg = startSeg;

            for (int i = 0; i < maxHops; i++)
            {
                GraphPatches.SegmentsReachableFrom(graph, seg, walkEnd,
                    out TrackSegment normal, out TrackSegment reversed);

                if (normal == target || reversed == target)
                    return true;

                // Continue along normal path only (don't branch at switches)
                if (normal == null) return false;

                var node = seg.NodeForEnd(walkEnd);
                if (node == null) return false;

                var otherNode = normal.GetOtherNode(node);
                if (otherNode == null) return false;

                walkEnd = normal.EndForNode(otherNode);
                seg = normal;
            }

            return false;
        }

        public static TrackNode FindSharedNode(TrackSegment a, TrackSegment b)
        {
            var aNodeA = a.NodeForEnd(TrackSegment.End.A);
            var aNodeB = a.NodeForEnd(TrackSegment.End.B);
            var bNodeA = b.NodeForEnd(TrackSegment.End.A);
            var bNodeB = b.NodeForEnd(TrackSegment.End.B);

            if (aNodeA != null && (aNodeA == bNodeA || aNodeA == bNodeB)) return aNodeA;
            if (aNodeB != null && (aNodeB == bNodeA || aNodeB == bNodeB)) return aNodeB;
            return null;
        }
    }
}
