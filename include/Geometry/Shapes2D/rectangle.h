#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "../shape.h"

#define DEF_WIDTH 1
#define DEF_HEIGHT 1

namespace geo {

  /** CLASS Rectangle
    * Template params:
    *                 T --> type used for the coordinates
    * A Rectangle is a 2D figure. Additional dimensions will be completed with coordinate value 0.0
    * The Rectangle is centered at the origin
    * Parameters:
    *            - width (default = 1)
    *            - height (default = 1)
    */

  template <typename T = float>
  class Rectangle : public Shape<T, 3>{
  protected:
    T _width;
    T _height;
    Point<T, 3> _center;

  public:
    Rectangle() : Rectangle(DEF_WIDTH, DEF_HEIGHT, Point<T, 3>()) {}

    Rectangle(T length) : Rectangle(length, length, Point<T, 3>()) {}

    Rectangle(T width, T height) : Rectangle(width, height, Point<T, 3>()) {}

    Rectangle(T width, T height, Point<T, 3> center) : _width{width}, _height{height}, _center{center} {
      this->_vertices.resize(4);
      this->_normals.resize(4);

      // Calculate the vertices starting from bottom-left (anticlockwise)
      this->_vertices[0][0] = center.x() - _width/static_cast<T>(2.0);
      this->_vertices[0][1] = center.y() - _height/static_cast<T>(2.0);
      this->_vertices[0][2] = center.z();
      this->_vertices[1][0] = center.x() + _width/static_cast<T>(2.0);
      this->_vertices[1][1] = center.y() - _height/static_cast<T>(2.0);
      this->_vertices[1][2] = center.z();
      this->_vertices[2][0] = center.x() + _width/static_cast<T>(2.0);
      this->_vertices[2][1] = center.y() + _height/static_cast<T>(2.0);
      this->_vertices[2][2] = center.z();
      this->_vertices[3][0] = center.x() - _width/static_cast<T>(2.0);
      this->_vertices[3][1] = center.x() + _height/static_cast<T>(2.0);
      this->_vertices[3][2] = center.z();

      // Normals to the sides, starting with the bottom side
      this->_normals[0][0] = 0.0;
      this->_normals[0][1] = static_cast<T>(-1.0);
      this->_normals[1][0] = static_cast<T>(1.0);
      this->_normals[1][1] = 0.0;
      this->_normals[2][0] = 0.0;
      this->_normals[2][1] = static_cast<T>(1.0);
      this->_normals[3][0] = static_cast<T>(-1.0);
      this->_normals[3][1] = 0.0;
    }

    Rectangle(const Rectangle& c) : Shape<T, 3>(c), _width{c._width}, _height{c._height}, _center{c._center} {}

    ~Rectangle(){};

    T width() const { return _width; }
    T height() const { return _height; }

    const Point<T, 3> & center() const { return _center; }

    T length() const { return Rectangle::length(_width, _height); }
    T area() const { return Rectangle::area(_width, _height); }
    T volume() const { return 0.0; }

    static T length(T width, T height) { return static_cast<T>(2.0) * (width + height); }
    static T area(T width, T height) { return  width * height; }

    virtual void rotate3D(T angle, Eigen::Matrix<T, 3, 1> axis){
      this->Shape<T, 3>::rotate3D(angle, axis);

      _center = Eigen::AngleAxis<T>(angle, axis) * _center;
    }

    friend std::ostream& operator<<(std::ostream& os, const Rectangle<T>& rec) {
      os << "{ *** Rectangle W=" << rec.width() << " H=" << rec.height() << " ***" << std::endl;
      os << " Center: " << rec.center();
      auto it = rec._vertices.begin();
      while(it != rec._vertices.end())
        os << *(it++);
      os << "}" << std::endl;

      return os;
    }

  }; // class Rectangle

} // namespace geo

#undef DEF_WIDTH
#undef DEF_HEIGHT

#endif // RECTANGLE_H
