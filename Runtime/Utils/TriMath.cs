using LatiosNavigation.Authoring;
using Unity.Mathematics;

namespace LatiosNavigation.Utils
{
    public static class TriMath
    {
        public static bool IsPointInTriangle(float3 point, NavTriangle triangle)
        {
            // Project to XZ plane
            var p_xz = new float2(point.x, point.z);
            var a_xz = new float2(triangle.pointA.x, triangle.pointA.z);
            var b_xz = new float2(triangle.pointB.x, triangle.pointB.z);
            var c_xz = new float2(triangle.pointC.x, triangle.pointC.z);

            // vectors from A to other vertices and point
            var v0_ac = c_xz - a_xz;
            var v1_ab = b_xz - a_xz;
            var v2_ap = p_xz - a_xz;

            // Compute dot products
            var dot00 = math.dot(v0_ac, v0_ac);
            var dot01 = math.dot(v0_ac, v1_ab);
            var dot02 = math.dot(v0_ac, v2_ap);
            var dot11 = math.dot(v1_ab, v1_ab);
            var dot12 = math.dot(v1_ab, v2_ap);

            // Barycentric coordinates
            var denominator = dot00 * dot11 - dot01 * dot01;

            // check for degenerate triangle
            if (math.abs(denominator) < 1e-6f) return false; // Degenerate triangle, cannot determine point inside

            var invDenom = 1f / denominator;
            var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is inside the triangle
            return u >= 0 && v >= 0 && u + v <= 1;
        }

        public static float TriArea2(float3 a, float3 b, float3 c)
        {
            // 2D signed area (XZ plane)
            var ax = b.x - a.x;
            var az = b.z - a.z;
            var bx = c.x - a.x;
            var bz = c.z - a.z;
            return bx * az - ax * bz;
        }


        public static float DistanceToTriangleSq(float3 point, NavTriangle triangle)
        {
            var r = triangle.Radius;
            var d = math.distance(point, triangle.Centroid);

            if (d > r)
                // Point is outside the bounding radius of the triangle
                return math.distancesq(point, triangle.Centroid);

            // Point is within the bounding radius, check if it's inside the triangle
            if (IsPointInTriangle(point, triangle))
                // Point is inside the triangle
                return 0f;

            return float.PositiveInfinity;
        }
    }
}