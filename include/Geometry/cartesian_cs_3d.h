#ifndef CARTESIAN_CS_3D_H
#define CARTESIAN_CS_3D_H

#include <exception>
#include <limits>

#include <Eigen/Geometry>

namespace geo {

  template <typename T = float>
  class CartesianCS_3D {
    std::array<Eigen::Matrix<T, 3, 1>, 3> _axis;

    Eigen::Matrix<T, 3, 1> _center {0, 0, 0};

    Eigen::Matrix<T, 4, 4> transf_matrix {Eigen::Matrix<T, 4, 4>::Identity()};
  public:
    /**
      * Constructor: DEFAULT CARTESIAN COORDINATE SYSTEM
      */
    CartesianCS_3D() : _axis{ Eigen::Matrix<T, 3, 1>::UnitX(), Eigen::Matrix<T, 3, 1>::UnitY(), Eigen::Matrix<T, 3, 1>::UnitZ() } {}

    /**
      * Constructor: ARBITRARY CARTESIAN COORDINATE SYSTEM. Based on 2 given orthogonal vectors (for axis "X" and "Y"). These 2 vectors don't have to be normalized.
      *   Axis Z is calculated
      */
    CartesianCS_3D(const Eigen::Matrix<T, 3, 1>& axis_1, const Eigen::Matrix<T, 3, 1>& axis_2) : CartesianCS_3D(Eigen::Matrix<T, 3, 1>(0, 0, 0), axis_1, axis_2) {}

    /**
      * Constructor: ARBITRARY CARTESIAN COORDINATE SYSTEM. Based on 2 given orthogonal vectors (for axis "X" and "Y") and a center. These 2 vectors don't have to be normalized.
      *   Axis Z is calculated
      */
    CartesianCS_3D(const Eigen::Matrix<T, 3, 1>& center, const Eigen::Matrix<T, 3, 1>& axis_1, const Eigen::Matrix<T, 3, 1>& axis_2) : _center{center}{
      _axis[0] = axis_1.normalized();
      _axis[1] = axis_2.normalized();

      // Validate Orthogonality
      if( abs(_axis[0].dot(_axis[1])) > static_cast<T>(10.0) * std::numeric_limits<T>::epsilon() )
        throw std::invalid_argument("Axis vectors must be orthogonal.");

      _axis[2] = _axis[0].cross(_axis[1]);

      // Calculate and store the transformation matrix
      transf_matrix <<  _axis[0].x(),             _axis[0].y(),             _axis[0].z(),             -_axis[0].dot(_center),
                        _axis[1].x(),             _axis[1].y(),             _axis[1].z(),             -_axis[1].dot(_center),
                        _axis[2].x(),             _axis[2].y(),             _axis[2].z(),             -_axis[2].dot(_center),
                              0,                        0,                        0,                      static_cast<T>(1);
    }


    ~CartesianCS_3D(){};


    const Eigen::Matrix<T, 3, 1> & center() const { return _center; }

    const Eigen::Matrix<T, 3, 1> & operator[](uint8_t pos) const { return _axis.at(pos); }


    /**
      * Returns the transformation matrix to go from coordinates in the default CS to this coordinates system.
      */
    inline const Eigen::Matrix<T, 4, 4>& transformMatrix() const { return transf_matrix; }


    /**
      * Returns the transformation matrix to go from coordinates in the default CS to coordinates represented by a new CS centered at "position",
      *     with axis Z in the opposite direction  to "look_at", and axis Y with the orientation determined by "vertical"
      */
    static inline Eigen::Matrix<T, 4, 4> transformMatrix(const Eigen::Matrix<T, 3, 1>& position, const Eigen::Matrix<T, 3, 1>& look_at, const Eigen::Matrix<T, 3, 1>& vertical) {
      Eigen::Matrix<T, 3, 1> newZ = position - look_at;
      newZ.normalize();
      Eigen::Matrix<T, 3, 1> newX = vertical.cross(newZ);
      newX.normalize();
      Eigen::Matrix<T, 3, 1> newY = newZ.cross(newX);

      return CartesianCS_3D<T>(position, newX, newY).transformMatrix();
    }


    friend std::ostream& operator<<(std::ostream& os, const CartesianCS_3D<T>& cs) {
      os << "{ " << std::endl;
      os << "  CENTER: { ";
      uint8_t i;
      for(i = 0; i < 2; i++){
        os << cs._center[i] << ", ";
      }
      os << cs._center[i] << " }" << std::endl;
      for(i = 0; i < 3; i++){
        os << "  { ";
        uint8_t j;
        for(j = 0; j < 2; j++)
          os << cs[i][j] << ", ";
        os << cs[i][j] << " }" << std::endl;
      }
      os << "}" << std::endl;

      return os;
    }

  }; // class CartesianCS_3D

} // namespace geo


#endif // CARTESIAN_CS_3D_H
