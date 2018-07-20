#ifndef CIRCLE_H
#define CIRCLE_H

#include <cmath>

#include "../constants.h"
#include "../shape.h"

#define DEF_RADIUS 1
#define DEF_NUM_VERTICES 36

namespace geo {

  /** CLASS Circle
    * Template params:
    *                 T --> type used for the coordinates
    * A circle is a 2D figure. Additional dimensions will be completed with coordinate value 0.0
    * The circle is centered at the origin
    * Parameters:
    *            - radius (default = 1)
    *            - num_vertices (default = 36, 1 vertex per 10°)
    */

  template <typename T = float>
  class Circle : public Shape<T, 3>{
  protected:
    T _radius;
    Point<T, 3> _center;

  public:
    Circle() : Circle(DEF_RADIUS, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Circle(T radius) : Circle(radius, Point<T, 3>(), DEF_NUM_VERTICES) {}

    Circle(T radius, size_t num_vertices) : Circle(radius, Point<T, 3>(), num_vertices) {}

    Circle(T radius, Point<T, 3> center, size_t num_vertices = DEF_NUM_VERTICES) : _radius{radius}, _center{center} {
      this->_vertices.resize(num_vertices);
      this->_normals.resize(this->_vertices.size());

      // Calculate the vertices starting from (+radius,0)
      T delta_angle  = static_cast<T>(_2PI_) / num_vertices;
      for (size_t i = 0; i < num_vertices; i++){
        this->_normals[i][0] = cos(delta_angle * i);
        this->_normals[i][1] = sin(delta_angle * i);
        for (uint8_t j = 2; j < 3; j++)
          this->_normals[i][j] = 0.0;

        this->_vertices[i] = center + radius * this->_normals[i];
      }
    }

    Circle(const Circle& c) : Shape<T, 3>(c), _radius{c._radius}, _center{c._center} {}

    ~Circle(){};

    T radius() const { return _radius; }

    const Point<T, 3> & center() const { return _center; }

    T length() const { return Circle::length(_radius); }
    T area() const { return Circle::area(_radius); }
    T volume() const { return 0.0; }

    static T length(T radius) { return static_cast<T>(_2PI_) * radius; }
    static T area(T radius) { return static_cast<T>(_PI_) * radius * radius; }

    virtual void rotate3D(T angle, Eigen::Matrix<T, 3, 1> axis){
      this->Shape<T, 3>::rotate3D(angle, axis);

      _center = Eigen::AngleAxis<T>(angle, axis) * _center;
    }

    friend std::ostream& operator<<(std::ostream& os, const Circle<T>& c) {
      os << "{ *** CIRCLE R=" << c.radius() << " (" << c.size() << " vertices) ***" << std::endl;
      os << " Center: " << c.center();
      auto it = c._vertices.begin();
      while(it != c._vertices.end())
        os << *(it++);
      os << "}" << std::endl;

      return os;
    }

  }; // class Circle

} // namespace geo

#undef DEF_RADIUS
#undef DEF_NUM_VERTICES

#endif // CIRCLE_H
