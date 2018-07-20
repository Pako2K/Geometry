#ifndef SHAPE_H
#define SHAPE_H

#include <vector>

#include "point.h"

namespace geo {

  /** ABSTRACT CLASS Shape
    * Template params:
    *                 T --> type used for the coordinates
    *                 DIM --> Number of dimensions
    */

  template <typename T = float, uint8_t DIM = 2>
  class Shape{

  protected:
    std::vector<Point<T, DIM>> _vertices;

    std::vector<Eigen::Matrix<T, DIM, 1>> _normals;

  public:
    Shape(){};

    Shape(const Shape& s) : _vertices{s._vertices}, _normals{s._normals} {};

    ~Shape(){}

    size_t size() const { return this->_vertices.size(); }

    const std::vector<Point<T, DIM>> & vertices() const { return this->_vertices; }
    const T* data() const { return this->_vertices.data()->data(); }

    const std::vector<Eigen::Matrix<T, DIM, 1>> & normals() const { return this->_normals; }
    const T* normalsData() const { return this->_normals.data()->data(); }

    virtual T area() const = 0;
    virtual T volume() const = 0;

    const Point<T, DIM>& operator[](size_t pos) const { return _vertices.at(pos); }

    void scale3D(T scale){
      scale3D(scale, scale, scale);
    }

    void scale3D(T scale_X, T scale_Y, T scale_Z){
      Eigen::Transform<T, 3, Eigen::Affine> scale_matrix(Eigen::Scaling(scale_X, scale_Y, scale_Z));

      typename std::vector<Point<T, 3>>::iterator vert_it = _vertices.begin();
      while(vert_it != _vertices.end()){
        *vert_it = scale_matrix * (*vert_it);
        vert_it++;
      }

      scale_matrix = scale_matrix.inverse();
      typename std::vector<Eigen::Matrix<T, 3, 1>>::iterator norm_it = _normals.begin();
      while(norm_it != _normals.end()){
        *norm_it = scale_matrix * (*norm_it);
        (*norm_it).normalize();
        norm_it++;
      }
    }

    void rotate2D(T angle) {
      if (DIM != 2) throw std::string("2D Rotation can only be applied to 2D Shapes");

      Eigen::Rotation2D<T> rotation_matrix(angle);

      typename std::vector<Point<T, 2>>::iterator vert_it = _vertices.begin();
      while(vert_it != _vertices.end()){
        *vert_it = rotation_matrix * (*vert_it);
        vert_it++;
      }

      typename std::vector<Eigen::Matrix<T, 2, 1>>::iterator norm_it = _normals.begin();
      while(norm_it != _normals.end()){
        *norm_it = rotation_matrix * (*norm_it);
        norm_it++;
      }
    }

    virtual void rotate3D(T angle, const Eigen::Matrix<T, 3, 1> & axis) {
      if (DIM != 3) throw std::string("3D Rotation can only be applied to 3D Shapes");

      Eigen::AngleAxis<T> rotation_matrix(angle, axis);

      typename std::vector<Point<T, 3>>::iterator vert_it = _vertices.begin();
      while(vert_it != _vertices.end()){
        *vert_it = rotation_matrix * (*vert_it);
        vert_it++;
      }

      typename std::vector<Eigen::Matrix<T, 3, 1>>::iterator norm_it = _normals.begin();
      while(norm_it != _normals.end()){
        *norm_it = rotation_matrix * (*norm_it);
        norm_it++;
      }
    }
  }; // class Shape

} // namespace geo


#endif // SHAPE_H
