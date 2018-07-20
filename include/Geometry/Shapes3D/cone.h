#ifndef CONE_H
#define CONE_H

#include <vector>
#include <cmath>

#include "../shape.h"
#include "../Shapes2D/circle.h"
#include "../point.h"

#define DEF_RADIUS 1
#define DEF_HEIGHT 1
#define DEF_NUM_VERTICES 36

namespace geo {

  /** CLASS Cone
    * Template params:
    *                 T --> type used for the coordinates
    *
    * A Cone is a 3D figure.
    * The circle is centered at the origin
    * Parameters:
    *            - radius (default = 1)
    *            - height (default = 1)
    *            - num_vertices (default = 36, 1 vertex per 10° in the base circle)
    */

  template <typename T = float>
  class Cone : public Shape<T, 3>{
  protected:
    T _radius;
    T _height;
    Point<T, 3> _base_center;

  public:
    Cone() : Cone(DEF_RADIUS, DEF_HEIGHT, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Cone(T radius, T height) : Cone(radius, height, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Cone(T radius, T height, size_t base_num_vertices) : Cone(radius, height, Point<T, 3>(), base_num_vertices) {}

    Cone(T radius, T height, const Point<T, 3>& base_center, size_t base_num_vertices = DEF_NUM_VERTICES) : _radius{radius}, _height{height}, _base_center{base_center}{
      Circle<T> base(radius, base_center, base_num_vertices);
      Point<T, 3> tip(base_center.x(), base_center.y(), base_center.z() +  height);

      // Reserve one additional vertex for the center of the base and
      // as many as base circle vertices for the tip (in order to be able to provide the right normals for the tip "singularity")
      this->_vertices.resize(base.size() + 1);
      this->_normals.resize(this->_vertices.size());

//      // Copy vertices data and calculate normals
//      this->_vertices[0] = base[0];
//      this->_normals[0] = (-1.0) * base.normals()[0];

      T xy_comp { static_cast<T>(1.0f) / std::sqrt(static_cast<T>(1.0f) + radius * radius / height / height) };
      T z_comp { radius / height * xy_comp };
      T delta_angle = _2PI_ / base_num_vertices;
      for(size_t i = 0; i < base_num_vertices; i++){
        this->_vertices[i] = base[i];
        this->_normals[i] = xy_comp * (Eigen::AngleAxis<T>(i * delta_angle, Eigen::Matrix<T, 3, 1>::UnitZ()) * Eigen::Matrix<T, 3, 1>::UnitX())
                            + z_comp * Eigen::Matrix<T, 3, 1>::UnitZ();
      }
      this->_vertices[base_num_vertices] = tip;
      this->_normals[base_num_vertices] = Eigen::Matrix<T, 3, 1>::UnitZ();
    }

    Cone(const Cone& c) : Shape<T, 3>(c), _radius{c._radius}, _height{c._height}, _base_center{c._base_center} {}

    ~Cone(){}

    T radius() const { return _radius; }
    T height() const { return _height; }
    const Point<T, 3> & base_center() const { return _base_center; }
    Eigen::Matrix<T, 3, 1> base_normal() const { return -Eigen::Matrix<T, 3, 1>::UnitZ(); }

    T area() const { return Cone::area(_radius, _height); }
    T volume() const { return Cone::volume(_radius, _height); }
    static T area(T radius, T height) { return static_cast<T>(_PI_) * radius * (radius + sqrt(radius * radius + height * height)); }
    static T volume(T radius, T height) { return static_cast<T>(_PI_) * radius * radius * height / 3; }

    virtual void rotate3D(T angle, const Eigen::Matrix<T, 3, 1> & axis){
      Shape<T, 3>::rotate3D(angle, axis);

      _base_center = Eigen::AngleAxis<T>(angle, axis) * _base_center;
    }

    friend std::ostream& operator<<(std::ostream& os, const Cone<T>& c) {
      os << "{ *** CONE R=" << c.radius() << " H=" << c.height() << " (" << c.size() << " vertices) ***" << std::endl;
      os << " Base Center: " << base_center();
      auto it = c._vertices.begin();
      while(it != c._vertices.end())
        os << *(it++);
      os << "}" << std::endl;

      return os;
    }
  }; // class Cone

} // namespace geo

#undef DEF_RADIUS
#undef DEF_HEIGHT
#undef DEF_NUM_VERTICES

#endif // CONE_H
