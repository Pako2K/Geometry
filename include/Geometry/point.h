#ifndef POINT_H
#define POINT_H

#include <ostream>
#include <iterator>

#include <Eigen/Geometry>

namespace geo {

  template <typename T = float, uint8_t DIM = 3>
  class Point : public Eigen::Matrix<T, DIM, 1> {
    static_assert(std::is_floating_point<T>::value, "Class type must be a floating number type");
    static_assert(DIM > 1, "Number of dimensions must be at least 2");

  public:
    Point(){
      for(uint8_t i = 0; i < DIM; i++)
        (*this)[i] = 0.0;
    }

    Point(T x) : Point() { (*this)[0] = x; }

    Point(T x, T y) : Point() {
      (*this)[0] = x;
      (*this)[1] = y;
    }

    Point(T x, T y, T z) : Point() {
      static_assert(DIM >= 3, "Constructor cannot be used: number of dimension must be at least 3");
      (*this)[0] = x;
      (*this)[1] = y;
      (*this)[2] = z;
    }

    template <typename InputIterator,
              typename = typename std::enable_if
                                    <std::is_convertible
                                      <typename std::iterator_traits<InputIterator>::iterator_category, std::input_iterator_tag>::value
                                    >::type
             >
    Point(InputIterator first, InputIterator last) : Point(){
      uint8_t pos {0};
      while(first!=last){
        //_point.at(pos++) = *first;
        (*this)[pos++] = *first;
        ++first;
      }
    }

    ~Point() {}

    T z() const {
      static_assert(DIM >= 3, "Member function cannot be used: number of dimension must be at least 3");
      return Eigen::Matrix<T, DIM, 1>::z();
    }

    Point<T, DIM>& operator=(T value){
      for(uint8_t i = 0; i < DIM; i++)
        (*this)[i] = value;

      return *this;
    }

    Point<T, DIM>& operator=(Eigen::Matrix<T, DIM, 1> vec){
      for(uint8_t i = 0; i < DIM; i++)
        (*this)[i] = vec[i];
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const Point<T, DIM>& p) {
      os << "{ ";
      uint8_t i;
      for(i = 0; i < DIM - 1; i++)
        os << p[i] << ", ";
      os << p[i] << " } " << std::endl;

      return os;
    }

  }; // class Point

} // namespace geo

#endif // POINT_H
