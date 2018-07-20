#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <Eigen/Geometry>

namespace geo {

  /**
    * Orthogonal projection
    * (Assumes that the frustrum input parameters are in the observer CS, except near and far which are absolute values in the direction of the view,
    *  which goes in the negative Z direction)
    */
  template <typename T>
  inline Eigen::Matrix<T, 4, 4> orthoProjection(T left, T right, T bottom, T top, T near, T far);

  /**
    * Perspective projection matrix
    * Input:
    *     field_angle: vertical field of view (in radians)
    *     field_ratio: field of view ratio (horizontal/vertical)
    *     near and far: absolute values in the direction of the view, which goes in the negative Z direction in the observer CS
    */
  template <typename T>
  inline Eigen::Matrix<T, 4, 4> perspectiveProjection(T field_angle, T field_ratio, T near, T far);





  template <typename T>
  inline Eigen::Matrix<T, 4, 4> orthoProjection(T left, T right, T bottom, T top, T near, T far){
    Eigen::Matrix<T, 4, 4> ortho;

    // Column major
    ortho << static_cast<T>(2) / (right - left),                  0,                                0,           - (right + left) / (right - left),
                        0,                      static_cast<T>(2) / (top - bottom),                 0,           - (top + bottom) / (top - bottom),
                        0,                                        0,              -static_cast<T>(1) / (far - near),    - near / (far - near),
                        0,                                        0,                                0,                   static_cast<T>(1);

    return ortho;
  }


  template <typename T>
  inline Eigen::Matrix<T, 4, 4> perspectiveProjection(T field_angle, T field_ratio, T near, T far){
    if( field_ratio < static_cast<T>(1) )
        throw std::invalid_argument("Field of view ratio must be grater than 1.");

    Eigen::Matrix<T, 4, 4> perspective;
    T tan_half_field = tan(field_angle / static_cast<T>(2));

    perspective << static_cast<T>(1) / (field_ratio * tan_half_field),        0,                                0,                           0,
                                  0,                        static_cast<T>(1) / (tan_half_field),               0,                           0,
                                  0,                                          0,                      -far / (far - near),      - near * far / (far - near),
                                  0,                                          0,                      -static_cast<T>(1),                    0;

    return perspective;
  }


} // namespace geo


#endif // TRANSFORMATIONS_H
