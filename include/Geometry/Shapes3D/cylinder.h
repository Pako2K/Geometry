#ifndef CYLINDER_H
#define CYLINDER_H

#include <vector>
#include <cmath>

#include "../shape.h"
#include "../Shapes2D/circle.h"
#include "../point.h"

#define DEF_RADIUS 1
#define DEF_HEIGHT 1
#define DEF_NUM_VERTICES 36

namespace geo {

  /** CLASS Cylinder
    * Template params:
    *                 T --> type used for the coordinates
    *
    * A Cylinder is a 3D figure.
    * The base circle is centered at the origin
    * VERTICES ORDER: first the base circle vertices, from x=R, y=0; then the top circle ones, in the same order
    * Parameters:
    *            - radius (default = 1)
    *            - height (default = 1)
    *            - base_num_vertices (default = 36, 1 vertex per degree in the base circle)
    */

  template <typename T = float>
  class Cylinder : public Shape<T, 3>{
  protected:
    T _radius;
    T _height;
    Point<T, 3> _base_center;
    Point<T, 3> _top_center;
    Eigen::Matrix<T, 3, 1> _base_normal;
    Eigen::Matrix<T, 3, 1> _top_normal;

  public:
    Cylinder() : Cylinder(DEF_RADIUS, DEF_HEIGHT, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Cylinder(T radius, T height) : Cylinder(radius, height, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Cylinder(T radius, T height, size_t base_num_vertices) : Cylinder(radius, height, Point<T, 3>(), base_num_vertices) {}

    Cylinder(T radius, T height, Point<T, 3> base_center, size_t base_num_vertices = DEF_NUM_VERTICES) :
            _radius{radius}, _height{height}, _base_center{base_center}, _top_center{Point<T, 3>(base_center.x(), base_center.y(), base_center.z() + height)},
            _base_normal{-Eigen::Matrix<T, 3, 1>::UnitZ()}, _top_normal{Eigen::Matrix<T, 3, 1>::UnitZ()} {
      Circle<T> base(radius, _base_center, base_num_vertices);
      Circle<T> top(radius, _top_center, base_num_vertices);

      // Reserve
      this->_vertices.resize(base.size() + top.size());
      this->_normals.resize(this->_vertices.size());

      // Copy data. First 2 vertices are the circle centers
      for(size_t i = 0; i < base.size(); i++){
        this->_vertices[i] = base[i];
        this->_vertices[base.size() + i] = top[i];

        this->_normals[i] = base.normals()[i];
        this->_normals[base.size() + i] = top.normals()[i];
      }
    }

    Cylinder(const Cylinder& c) : Shape<T, 3>(c), _radius{c._radius}, _height{c._height}, _base_center{c._base_center}, _top_center{_top_center} {}

    ~Cylinder(){}

    T radius() const { return _radius; }
    T height() const { return _height; }
    const Point<T, 3> & base_center() const { return _base_center; }
    Eigen::Matrix<T, 3, 1> base_normal() const { return _base_normal; }
    const Point<T, 3> & top_center() const { return _top_center; }
    Eigen::Matrix<T, 3, 1> top_normal() const { return _top_normal; }

    T area() const { return Cylinder::area(_radius, _height); }
    T volume() const { return Cylinder::volume(_radius, _height); }
    static T area(T radius, T height) { return static_cast<T>(_2PI_) * radius * (radius + height); }
    static T volume(T radius, T height) { return static_cast<T>(_PI_) * radius * radius * height; }

    virtual void rotate3D(T angle, Eigen::Matrix<T, 3, 1> axis){
      Shape<T, 3>::rotate3D(angle, axis);

      _base_center = Eigen::AngleAxis<T>(angle, axis) * _base_center;
      _top_center = Eigen::AngleAxis<T>(angle, axis) * _top_center;
      _base_normal = Eigen::AngleAxis<T>(angle, axis) * _base_normal;
      _top_normal = Eigen::AngleAxis<T>(angle, axis) * _top_normal;
    }

    friend std::ostream& operator<<(std::ostream& os, const Cylinder<T>& c) {
      os << "{ *** CYLINDER R=" << c.radius() << " H=" << c.height() << " (" << c.size() << " vertices) ***" << std::endl;
      os << " Base Center: " << c.base_center();
      os << " Top Center: " << c.top_center();
      auto it = c._vertices.begin();
      while(it != c._vertices.end())
        os << *(it++);
      os << "}" << std::endl;

      return os;
    }
  }; // class Cylinder

} // namespace geo

#undef DEF_RADIUS
#undef DEF_HEIGHT
#undef DEF_NUM_VERTICES

#endif // CYLINDER_H
