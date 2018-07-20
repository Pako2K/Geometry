#ifndef CUBOID_H
#define CUBOID_H

#include "../shape.h"
#include "../Shapes2D/rectangle.h"

#define DEF_WIDTH 1
#define DEF_HEIGHT 1
#define DEF_DEPTH 1

namespace geo {

  /** CLASS Cuboid
    * Template params:
    *                 T --> type used for the coordinates
    * A Cuboid is a 3D figure.
    * The Cuboid is centered at the origin
    * Parameters:
    *            - width (default = 1)
    *            - height (default = 1)
    *            - depth (default = 1)
    */

  template <typename T = float>
  class Cuboid : public Shape<T, 3>{
  protected:
    T _width;
    T _height;
    T _depth;
    Point<T, 3> _center;

  public:
    Cuboid() : Cuboid(DEF_WIDTH, DEF_HEIGHT, DEF_DEPTH, Point<T, 3>()) {}

    Cuboid(T length) : Cuboid(length, length, length, Point<T, 3>()) {}

    Cuboid(T width, T height, T depth) : Cuboid(width, height, depth, Point<T, 3>()) {}

    Cuboid(T width /*X*/, T height /*Z*/, T depth /*Y*/, Point<T, 3> center) : _width{width}, _height{height}, _depth{depth}, _center{center} {
      this->_vertices.resize(8);
      this->_normals.resize(6);

      // Calculate the vertices starting from bottom-left (anticlockwise)
      //    Bottom & up Faces
      Rectangle<T> bottom(_width, _depth, Point<T, 3> (_center.x(), _center.y(), _center.z() - height / static_cast<T>(2.0)));
      Rectangle<T> up(_width, _depth, Point<T, 3> (_center.x(), _center.y(), _center.z() + height / static_cast<T>(2.0)));

      for (uint8_t i = 0; i < 4; i++){
        this->_vertices[i] = bottom.vertices()[i];
        this->_vertices[i + 4] = up.vertices()[i];
      }

      // Normals to the sides, starting with the bottom side
      this->_normals[0] = Eigen::Matrix<T, 3, 1>(0, 0, static_cast<T>(-1.0));
      for (uint8_t i = 0; i < 4; i++)
        this->_normals[i + 1] = bottom.normals()[i];
      this->_normals[5] = Eigen::Matrix<T, 3, 1>(0, 0, static_cast<T>(1.0));
    }

    Cuboid(const Cuboid& c) : Shape<T, 3>(c), _width{c._width}, _height{c._height}, _depth{c._depth}, _center{c._center} {}

    ~Cuboid(){};

    T width() const { return _width; }
    T height() const { return _height; }
    T depth() const { return _depth; }

    const Point<T, 3> & center() const { return _center; }

    T area() const { return Cuboid::area(_width, _height, _depth); }
    T volume() const { return Cuboid::volume(_width, _height, _depth); }

    static T area(T width, T height, T depth) { return  static_cast<T>(2.0) * (width * depth + width * height + depth * height); }
    static T volume(T width, T height, T depth) { return  width * depth * height; }

    virtual void rotate3D(T angle, Eigen::Matrix<T, 3, 1> axis){
      this->Shape<T, 3>::rotate3D(angle, axis);

      _center = Eigen::AngleAxis<T>(angle, axis) * _center;
    }

    friend std::ostream& operator<<(std::ostream& os, const Cuboid<T>& cub) {
      os << "{ *** Cuboid W=" << cub.width() << " D=" << cub.depth() << " H=" << cub.height() << " ***" << std::endl;
      os << " Center: " << cub.center();
      auto it = cub._vertices.begin();
      while(it != cub._vertices.end())
        os << *(it++);
      os << "}" << std::endl;

      return os;
    }

  }; // class Cuboid

} // namespace geo

#undef DEF_WIDTH
#undef DEF_HEIGHT
#undef DEF_DEPTH


#endif // CUBOID_H
