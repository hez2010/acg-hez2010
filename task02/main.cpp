#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // rotation angle
  const float theta = -atan2(dir.y(), dir.x());
  // rotation matrix
  const Eigen::Matrix3f rotation =
    (Eigen::Matrix3f() <<
      cos(theta), -sin(theta), 0,
      sin(theta), cos(theta),  0,
      0,          0,           1)
    .finished();
  // translation matrix
  const Eigen::Matrix3f translation =
    (Eigen::Matrix3f() <<
      1, 0, -org.x(),
      0, 1, -org.y(),
      0, 0, 1)
    .finished();
  // transform with homogeneous coordinates so that org = (0, 0), dir = (1, 0)
  const Eigen::Vector3f org_h = rotation * (translation * (Eigen::Vector3f() << org, 1).finished());
  const Eigen::Vector3f dir_h = rotation * (translation * (Eigen::Vector3f() << dir.normalized(), 0).finished());
  const Eigen::Vector3f ps_h = rotation * (translation * (Eigen::Vector3f() << ps, 1).finished());
  const Eigen::Vector3f pc_h = rotation * (translation * (Eigen::Vector3f() << pc, 1).finished());
  const Eigen::Vector3f pe_h = rotation * (translation * (Eigen::Vector3f() << pe, 1).finished());
  // solving (1 - t)^2 * ps + 2 * t * (1 - t) * pc + t^2 * pe = org + s * dir
  // => (ps - 2 * pc + pe) * t^2 + 2 * (pc - ps) * t + ps = org + s * dir
  // => a * t^2 + b * t + c = (s, 0)
  // where t > 0 and < 1, s > 0
  const Eigen::Vector2f a = (ps_h - 2 * pc_h + pe_h).head(2);
  const Eigen::Vector2f b = (2 * (pc_h - ps_h)).head(2);
  const Eigen::Vector2f c = (ps_h).head(2);
  const float discriminant = b.y() * b.y() - 4 * a.y() * c.y();
  // d < 0, no intersection
  if (discriminant < 0)
    return 0;
  const float t1 = (-b.y() + sqrt(discriminant)) / (2 * a.y());
  const float t2 = (-b.y() - sqrt(discriminant)) / (2 * a.y());
  const float s1 = a.x() * t1 * t1 + b.x() * t1 + c.x();
  const float s2 = a.x() * t2 * t2 + b.x() * t2 + c.x();
  // d == 0, maybe one intersection
  if (abs(discriminant) < std::numeric_limits<float>::epsilon()) {
    if (t1 > 0 && t1 < 1 && s1 > 0) {
      return 1;
    }
    return 0;
  }
  // d > 0, maybe two intersections
  int count = 0;
  if (t1 > 0 && t1 < 1 && s1 > 0) {
    count += 1;
  }
  if (t2 > 0 && t2 < 1 && s2 > 0)
    count += 1;
  return count;
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
