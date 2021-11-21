#include "geometry.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

using Polygon = std::vector<float3>;

bool VERBOSE = false;

/* Checks if a given point is 'inside' the clipping polygon (i.e. if the point is to the right of
 * the clippling line)
 * Adapted from python implementation at https://github.com/mdabdk/sutherland-hodgman
 */
bool is_inside(const float3& p1, const float3& p2, const float3& q)
{
    float R = (p2.x - p1.x) * (q.y - p1.y) - (p2.y - p1.y) * (q.x - p1.x);
    return R <= 0.f;
}

/* Computes the intersection between two lines.
 * Adapted from python implementation at https://github.com/mdabdk/sutherland-hodgman
 */
float3 compute_intersection(
    const float3& p1, const float3& p2, const float3& p3, const float3& p4)
{
    float3 intersection{};
    // if first line is vertical
    if (p2.x == p1.x)
    {
        intersection.x = p1.x;
        
        // slope and intercept of second line
        float a2 = (p4.y - p3.y) / (p4.x - p3.x);
        float b2 = p3.y - a2 * p3.x;
        
        intersection.y = a2 * intersection.x + b2;
    }
    // if second line is vertical
    else if (p4.x == p3.x)
    {
        intersection.x = p3.x;
        
        // slope and intercept of first line
        float a1 = (p2.y - p1.y) / (p2.x - p1.x);
        float b1 = p1.y - a1 * p1.x;
        
        intersection.y = a1 * intersection.x + b1;
    }
    // if neither line is vertical
    else
    {
        float a1 = (p2.y - p1.y) / (p2.x - p1.x);
        float b1 = p1.y - a1 * p1.x;
        
        // slope and intercept of second line
        float a2 = (p4.y - p3.y) / (p4.x - p3.x);
        float b2 = p3.y - a2 * p3.x;
    
        intersection.x = (b2 - b1) / (a1 - a2);
        intersection.y = a1 * intersection.x + b1;
    }
    return intersection;
}

/* Translate and rotate two coplanar triangles in order to be placed at XY plane
 * source: J. Heller. Mathematics Stack Exchange, https://math.stackexchange.com/q/2041383
 */
void move_to_xy_plane(Triangle& home_triangle, Triangle& near_triangle)
{    
    // skip rotation if is it already parallel to the xy plane
    if (fabs(dot(normalize(home_triangle.normal()), float3{0, 0, 1})) == 1.f)
        return;

    // translate first triangle's vertex 0 to origin, then translate other vertices accordingly
    near_triangle.translate(-home_triangle.v0);
    home_triangle.translate(-home_triangle.v0);

    // To rotate the plane to the xy plane, all you need to do is rotate n to (0,0,1). The simplest 
    // way to do this is to first rotate n about the z axis into the positive x half of the xz plane
    float3 n = normalize(home_triangle.normal());
    const float xy_norm = sqrt(n.x * n.x + n.y * n.y);
    const float Rz[3][3] = {
        {n.x / xy_norm, n.y / xy_norm, 0},
        {- n.y / xy_norm, n.x / xy_norm, 0},
        {0, 0, 1}
    };
    home_triangle.rotate(Rz);
    near_triangle.rotate(Rz);

    // and then rotate about the y axis until n is parallel to the positive z axis.
    n = normalize(home_triangle.normal());
    const float Ry[3][3] = {
        {n.z, 0, -n.x},
        {0, 1, 0},
        {n.x, 0, n.z}
    };
    home_triangle.rotate(Ry);
    near_triangle.rotate(Ry);
}

/* Clip a triangle by using another triangle
 * Ivan Sutherland, Gary W. Hodgman: Reentrant Polygon Clipping. Communications of the ACM, vol. 17, pp. 32–42, 1974
 * Adapted from python implementation at https://github.com/mdabdk/sutherland-hodgman
 */
Polygon clip(Triangle home_triangle, Triangle near_triangle)
{
    move_to_xy_plane(home_triangle, near_triangle);

    // vertices need to be in clockwise order to use this algorithm
    auto is_counter_clockwise = [](const Triangle& t) -> bool
    {
        return ((t.v1.x - t.v0.x) * (t.v2.y - t.v0.y) - (t.v2.x - t.v0.x) * (t.v1.y - t.v0.y)) > 0;
    };
    if (is_counter_clockwise(home_triangle))
        std::swap(home_triangle.v1, home_triangle.v2);
    if (is_counter_clockwise(near_triangle))
        std::swap(near_triangle.v1, near_triangle.v2);

    std::ofstream f;
    if (VERBOSE)
    {
        f.open("triangles.obj");
        f << "o t0\n";
        f << "v " << home_triangle.v0.x << " " << home_triangle.v0.y << " " << home_triangle.v0.z << "\n";
        f << "v " << home_triangle.v1.x << " " << home_triangle.v1.y << " " << home_triangle.v1.z << "\n";
        f << "v " << home_triangle.v2.x << " " << home_triangle.v2.y << " " << home_triangle.v2.z << "\n";
        f << "f 1 2 3\n";
        f << "o t1\n";
        f << "v " << near_triangle.v0.x << " " << near_triangle.v0.y << " " << near_triangle.v0.z << "\n";
        f << "v " << near_triangle.v1.x << " " << near_triangle.v1.y << " " << near_triangle.v1.z << "\n";
        f << "v " << near_triangle.v2.x << " " << near_triangle.v2.y << " " << near_triangle.v2.z << "\n";
        f << "f 4 5 6\n";
    }

    const Polygon subject_poly { home_triangle.v0, home_triangle.v1, home_triangle.v2 };
    const Polygon clipping_poly { near_triangle.v0, near_triangle.v1, near_triangle.v2 };
    Polygon final_poly(subject_poly.begin(), subject_poly.end());

    for (int i = 0; i < clipping_poly.size(); i++)
    {
        // stores the vertices of the next iteration of the clipping procedure
        Polygon next_poly(final_poly.begin(), final_poly.end());
        // stores the vertices of the final clipped polygon
        final_poly.clear();

        // these two vertices define a line segment (edge) in the clipping polygon.
        const int prev_i = (i > 0) ? i - 1 : clipping_poly.size() - 1;
        const float3& c_edge_start = clipping_poly[prev_i];
        const float3& c_edge_end = clipping_poly[i];

        for (int j = 0; j < next_poly.size(); j++)
        {
            // these two vertices define a line segment (edge) in the subject polygon
            const int prev_j = (j > 0) ? j - 1 : next_poly.size() - 1;
            const float3& s_edge_start = next_poly[prev_j];
            const float3& s_edge_end = next_poly[j];
            
            if (is_inside(c_edge_start, c_edge_end, s_edge_end))
            {
                if (!is_inside(c_edge_start, c_edge_end, s_edge_start))
                {
                    float3 intersection = compute_intersection(
                        s_edge_start, s_edge_end, c_edge_start, c_edge_end);
                    final_poly.push_back(intersection);
                }
                final_poly.push_back(s_edge_end);
            }
            else if (is_inside(c_edge_start, c_edge_end, s_edge_start))
            {
                float3 intersection = compute_intersection(
                    s_edge_start, s_edge_end, c_edge_start, c_edge_end);
                final_poly.push_back(intersection);
            }
        }
    }

    if (VERBOSE)
    {
        f << "o poly\n";
        float3 center{};
        for (auto v : final_poly)
            center += v;
        center = center / final_poly.size();
        f << "v " << center.x << " " << center.y << " " << center.z << "\n";
        for (auto v : final_poly)
            f << "v " << v.x << " " << v.y << " " << v.z << "\n";
        for (int i = 0; i < final_poly.size(); i++)
            f << "f " << i + 8  << " " << ((i + 1) % final_poly.size()) + 8 << " 7\n";
        f.close();
    }

    return final_poly;
}

/* Computes the area of a polygon on xy plane.
 * The coordinates must be taken in counterclockwise order around the polygon, beginning and 
 * ending at the same point. As original polygon was in clockwise order, we just need to reverse 
 * iterate.
 * Source: http://www.mathwords.com/a/area_convex_polygon.htm
 */
double poly_area(const Polygon& poly)
{
    double left = 0.f, right = 0.f;
    for (auto it = poly.rbegin(); it != poly.rend(); it++)
    {
        auto next_it = (std::next(it) != poly.rend()) ? std::next(it) : poly.rbegin();
        left += it->x * next_it->y;
        right += it->y * next_it->x;
    }
    return (left - right) / 2.f;
}

int main(int argc, char* argv[])
{
    Triangle A, B;
    double expected_area;

    std::cin >> A.v0.x >> A.v0.y >> A.v0.z;
    std::cin >> A.v1.x >> A.v1.y >> A.v1.z;
    std::cin >> A.v2.x >> A.v2.y >> A.v2.z;
    std::cin >> B.v0.x >> B.v0.y >> B.v0.z;
    std::cin >> B.v1.x >> B.v1.y >> B.v1.z;
    std::cin >> B.v2.x >> B.v2.y >> B.v2.z;
    std::cin >> expected_area;

    int n_runs = 1;
    if (argc > 1)
    {
        n_runs = std::atoi(argv[1]);
        if (argc > 2 && std::string(argv[2]) == "-v")
            VERBOSE = true;
    }

    double area = 0;
    for (int i = 0; i < n_runs; i++)
    {
        Polygon result = std::move(clip(A, B));
        double _area = poly_area(result);
        if (i == 0)
            area = _area;
    }
    std::cout << "Computed area: " << std::setprecision(16) << area << std::endl;
    std::cout << "Expected area: " << std::setprecision(16) << expected_area << std::endl;
    
    double difference = fabs(area - expected_area) / expected_area;
    if (difference > 1e-8)
    {
        std::cerr << "Difference: " << std::setprecision(16) << difference * 100 << "%" << std::endl;
        return 1;
    }
    return 0;
}