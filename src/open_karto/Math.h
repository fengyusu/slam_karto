/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OPEN_KARTO_MATH_H
#define OPEN_KARTO_MATH_H

#include <assert.h>
#include <math.h>
#include <limits>
#include <cstddef>
#include <string>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>


#include "open_karto/Types.h"
#include "open_karto/Exception.h"

namespace karto
{
  /**
   * Platform independent pi definitions
   */
  const kt_double KT_PI         =  3.14159265358979323846;  // The value of PI
  const kt_double KT_2PI        =  6.28318530717958647692;  // 2 * PI
  const kt_double KT_PI_2       =  1.57079632679489661923;  // PI / 2
  const kt_double KT_PI_180     =  0.01745329251994329577;  // PI / 180
  const kt_double KT_180_PI     = 57.29577951308232087685;  // 180 / PI

  /**
   * Lets define a small number!
   */
  const kt_double KT_TOLERANCE  = 1e-06;

  /**
     * Lets define max value of kt_int32s (int32_t) to use it to mark invalid scans
     */

  const kt_int32s INVALID_SCAN = std::numeric_limits<kt_int32s>::max();




  namespace math
  {
    /**
     * Converts degrees into radians
     * @param degrees
     * @return radian equivalent of degrees
     */
    inline kt_double DegreesToRadians(kt_double degrees)
    {
      return degrees * KT_PI_180;
    }

    /**
     * Converts radians into degrees
     * @param radians
     * @return degree equivalent of radians
     */
    inline kt_double RadiansToDegrees(kt_double radians)
    {
      return radians * KT_180_PI;
    }

    /**
     * Square function
     * @param value
     * @return square of value
     */
    template<typename T>
    inline T Square(T value)
    {
      return (value * value);
    }

    /**
     * Round function
     * @param value
     * @return rounds value to the nearest whole number (as double)
     */
    inline kt_double Round(kt_double value)
    {
      return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
    }

    /**
     * Binary minimum function
     * @param value1
     * @param value2
     * @return the lesser of value1 and value2
     */
    template<typename T>
    inline const T& Minimum(const T& value1, const T& value2)
    {
      return value1 < value2 ? value1 : value2;
    }

    /**
     * Binary maximum function
     * @param value1
     * @param value2
     * @return the greater of value1 and value2
     */
    template<typename T>
    inline const T& Maximum(const T& value1, const T& value2)
    {
      return value1 > value2 ? value1 : value2;
    }

    /**
     * Clips a number to the specified minimum and maximum values.
     * @param n number to be clipped
     * @param minValue minimum value
     * @param maxValue maximum value
     * @return the clipped value
     */
    template<typename T>
    inline const T& Clip(const T& n, const T& minValue, const T& maxValue)
    {
      return Minimum(Maximum(n, minValue), maxValue);
    }

    /**
     * Checks whether two numbers are equal within a certain tolerance.
     * @param a
     * @param b
     * @return true if a and b differ by at most a certain tolerance.
     */
    inline kt_bool DoubleEqual(kt_double a, kt_double b)
    {
      double delta = a - b;
      return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
    }

    /**
     * Checks whether value is in the range [0;maximum)
     * @param value
     * @param maximum
     */
    template<typename T>
    inline kt_bool IsUpTo(const T& value, const T& maximum)
    {
      return (value >= 0 && value < maximum);
    }

    /**
     * Checks whether value is in the range [0;maximum)
     * Specialized version for unsigned int (kt_int32u)
     * @param value
     * @param maximum
     */
    template<>
    inline kt_bool IsUpTo<kt_int32u>(const kt_int32u& value, const kt_int32u& maximum)
    {
      return (value < maximum);
    }


    /**
     * Checks whether value is in the range [a;b]
     * @param value
     * @param a
     * @param b
     */
    template<typename T>
    inline kt_bool InRange(const T& value, const T& a, const T& b)
    {
      return (value >= a && value <= b);
    }

    /**
     * Normalizes angle to be in the range of [-pi, pi]
     * @param angle to be normalized
     * @return normalized angle
     */
    inline kt_double NormalizeAngle(kt_double angle)
    {
      while (angle < -KT_PI)
      {
        if (angle < -KT_2PI)
        {
          angle += (kt_int32u)(angle / -KT_2PI) * KT_2PI;
        }
        else
        {
          angle += KT_2PI;
        }
      }

      while (angle > KT_PI)
      {
        if (angle > KT_2PI)
        {
          angle -= (kt_int32u)(angle / KT_2PI) * KT_2PI;
        }
        else
        {
          angle -= KT_2PI;
        }
      }

      assert(math::InRange(angle, -KT_PI, KT_PI));

      return angle;
    }

    /**
     * Returns an equivalent angle to the first parameter such that the difference
     * when the second parameter is subtracted from this new value is an angle
     * in the normalized range of [-pi, pi], i.e. abs(minuend - subtrahend) <= pi.
     * @param minuend
     * @param subtrahend
     * @return normalized angle
     */
    inline kt_double NormalizeAngleDifference(kt_double minuend, kt_double subtrahend)
    {
      while (minuend - subtrahend < -KT_PI)
      {
        minuend += KT_2PI;
      }

      while (minuend - subtrahend > KT_PI)
      {
        minuend -= KT_2PI;
      }

      return minuend;
    }

    /**
     * Align a value to the alignValue.
     * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
     * @param value
     * @param alignValue
     * @return aligned value
     */
    template<class T>
    inline T AlignValue(size_t value, size_t alignValue = 8)
    {
      return static_cast<T> ((value + (alignValue - 1)) & ~(alignValue - 1));
    }
  }  // namespace math





  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a size (width, height) in 2-dimensional real space.
   */
  template<typename T>
  class Size2
  {
  public:
    /**
     * Default constructor
     */
    Size2()
      : m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * Constructor initializing point location
     * @param width
     * @param height
     */
    Size2(T width, T height)
      : m_Width(width)
      , m_Height(height)
    {
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Size2(const Size2& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Gets the width
     * @return the width
     */
    inline const T GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width
     * @param width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Gets the height
     * @return the height
     */
    inline const T GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height
     * @param height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Assignment operator
     */
    inline Size2& operator = (const Size2& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;

      return(*this);
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Size2& rOther) const
    {
      return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Size2& rOther) const
    {
      return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
    }

    /**
     * Write Size2 onto output stream
     * @param rStream output stream
     * @param rSize to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Size2& rSize)
    {
      rStream << "(" << rSize.m_Width << ", " << rSize.m_Height << ")";
      return rStream;
    }

  private:
    T m_Width;
    T m_Height;
  };  // Size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a vector (x, y) in 2-dimensional real space.
   */
  template<typename T>
  class Vector2
  {
  public:
    /**
     * Default constructor
     */
    Vector2()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
    }

    /**
     * Constructor initializing vector location
     * @param x
     * @param y
     */
    Vector2(T x, T y)
    {
      m_Values[0] = x;
      m_Values[1] = y;
    }

  public:
    /**
     * Gets the x-coordinate of this vector2
     * @return the x-coordinate of the vector2
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the x-coordinate of this vector2
     * @param x the x-coordinate of the vector2
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }

    /**
     * Gets the y-coordinate of this vector2
     * @return the y-coordinate of the vector2
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the y-coordinate of this vector2
     * @param y the y-coordinate of the vector2
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }

    /**
     * Floor point operator
     * @param rOther
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if ( rOther.m_Values[0] < m_Values[0] ) m_Values[0] = rOther.m_Values[0];
      if ( rOther.m_Values[1] < m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
     * Ceiling point operator
     * @param rOther
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if ( rOther.m_Values[0] > m_Values[0] ) m_Values[0] = rOther.m_Values[0];
      if ( rOther.m_Values[1] > m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]);
    }

    /**
     * Returns the length of the vector (x and y).
     * @return length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns the square distance to the given vector
     * @returns square distance to the given vector
     */
    inline kt_double SquaredDistance(const Vector2& rOther) const
    {
      return (*this - rOther).SquaredLength();
    }

    /**
     * Gets the distance to the other vector2
     * @param rOther
     * @return distance to other vector2
     */
    inline kt_double Distance(const Vector2& rOther) const
    {
      return sqrt(SquaredDistance(rOther));
    }

  public:
    /**
     * In place Vector2 addition.
     */
    inline void operator += (const Vector2& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
    }

    /**
     * In place Vector2 subtraction.
     */
    inline void operator -= (const Vector2& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
    }

    /**
     * Addition operator
     * @param rOther
     * @return vector resulting from adding this vector with the given vector
     */
    inline const Vector2 operator + (const Vector2& rOther) const
    {
      return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
    }

    /**
     * Subtraction operator
     * @param rOther
     * @return vector resulting from subtracting this vector from the given vector
     */
    inline const Vector2 operator - (const Vector2& rOther) const
    {
      return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
    }

    /**
     * In place scalar division operator
     * @param scalar
     */
    inline void operator /= (T scalar)
    {
      m_Values[0] /= scalar;
      m_Values[1] /= scalar;
    }

    /**
     * Divides a Vector2
     * @param scalar
     * @return scalar product
     */
    inline const Vector2 operator / (T scalar) const
    {
      return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
    }

    /**
     * Computes the dot product between the two vectors
     * @param rOther
     * @return dot product
     */
    inline kt_double operator * (const Vector2& rOther) const
    {
      return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
    }

    /**
     * Scales the vector by the given scalar
     * @param scalar
     */
    inline const Vector2 operator * (T scalar) const
    {
      return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
    }

    /**
     * Subtract the vector by the given scalar
     * @param scalar
     */
    inline const Vector2 operator - (T scalar) const
    {
      return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
    }

    /**
     * In place scalar multiplication operator
     * @param scalar
     */
    inline void operator *= (T scalar)
    {
      m_Values[0] *= scalar;
      m_Values[1] *= scalar;
    }

    /**
     * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Vector2& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Vector2& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
    }

    /**
     * Less than operator
     * @param rOther
     * @return true if left vector is less than right vector
     */
    inline kt_bool operator < (const Vector2& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
        return true;
      else if (m_Values[0] > rOther.m_Values[0])
        return false;
      else
        return (m_Values[1] < rOther.m_Values[1]);
    }

    /**
     * Write Vector2 onto output stream
     * @param rStream output stream
     * @param rVector to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector2& rVector)
    {
      rStream << rVector.GetX() << " " << rVector.GetY();
      return rStream;
    }

    /**
     * Read Vector2 from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Vector2& /*rVector*/)
    {
      // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement this?
      return rStream;
    }

  private:
    T m_Values[2];
  };  // Vector2<T>


  /**
   * Type declaration of Vector2<kt_double> vector
   */
  typedef std::vector< Vector2<kt_double> > PointVectorDouble;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a vector (x, y, z) in 3-dimensional real space.
   */
  template<typename T>
  class Vector3
  {
  public:
    /**
     * Default constructor
     */
    Vector3()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
      m_Values[2] = 0;
    }

    /**
     * Constructor initializing point location
     * @param x
     * @param y
     * @param z
     */
    Vector3(T x, T y, T z)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Vector3(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
    }

  public:
    /**
     * Gets the x-component of this vector
     * @return x-component
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the x-component of this vector
     * @param x
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }

    /**
     * Gets the y-component of this vector
     * @return y-component
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the y-component of this vector
     * @param y
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }

    /**
     * Gets the z-component of this vector
     * @return z-component
     */
    inline const T& GetZ() const
    {
      return m_Values[2];
    }

    /**
     * Sets the z-component of this vector
     * @param z
     */
    inline void SetZ(const T& z)
    {
      m_Values[2] = z;
    }

    /**
     * Floor vector operator
     * @param rOther Vector3d
     */
    inline void MakeFloor(const Vector3& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] < m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Ceiling vector operator
     * @param rOther Vector3d
     */
    inline void MakeCeil(const Vector3& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] > m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] > m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]) + math::Square(m_Values[2]);
    }

    /**
     * Returns the length of the vector.
     * @return Length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline std::string ToString() const
    {
      std::stringstream converter;
      converter.precision(std::numeric_limits<double>::digits10);

      converter << GetX() << " " << GetY() << " " << GetZ();

      return converter.str();
    }

  public:
    /**
     * Assignment operator
     */
    inline Vector3& operator = (const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];

      return *this;
    }

    /**
     * Binary vector add.
     * @param rOther
     * @return vector sum
     */
    inline const Vector3 operator + (const Vector3& rOther) const
    {
      return Vector3(m_Values[0] + rOther.m_Values[0],
                     m_Values[1] + rOther.m_Values[1],
                     m_Values[2] + rOther.m_Values[2]);
    }

    /**
     * Binary vector add.
     * @param scalar
     * @return sum
     */
    inline const Vector3 operator + (kt_double scalar) const
    {
      return Vector3(m_Values[0] + scalar,
                     m_Values[1] + scalar,
                     m_Values[2] + scalar);
    }

    /**
     * Binary vector subtract.
     * @param rOther
     * @return vector difference
     */
    inline const Vector3 operator - (const Vector3& rOther) const
    {
      return Vector3(m_Values[0] - rOther.m_Values[0],
                     m_Values[1] - rOther.m_Values[1],
                     m_Values[2] - rOther.m_Values[2]);
    }

    /**
     * Binary vector subtract.
     * @param scalar
     * @return difference
     */
    inline const Vector3 operator - (kt_double scalar) const
    {
      return Vector3(m_Values[0] - scalar, m_Values[1] - scalar, m_Values[2] - scalar);
    }

    /**
     * Scales the vector by the given scalar
     * @param scalar
     */
    inline const Vector3 operator * (T scalar) const
    {
      return Vector3(m_Values[0] * scalar, m_Values[1] * scalar, m_Values[2] * scalar);
    }

    /**
     * Equality operator returns true if the corresponding x, y, z values of each Vector3 are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Vector3& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] &&
              m_Values[1] == rOther.m_Values[1] &&
              m_Values[2] == rOther.m_Values[2]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y, z values of each Vector3 not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Vector3& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] ||
              m_Values[1] != rOther.m_Values[1] ||
              m_Values[2] != rOther.m_Values[2]);
    }

    /**
     * Write Vector3 onto output stream
     * @param rStream output stream
     * @param rVector to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector3& rVector)
    {
      rStream << rVector.ToString();
      return rStream;
    }

    /**
     * Read Vector3 from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Vector3& /*rVector*/)
    {
      // Implement me!!
      return rStream;
    }

  private:
    T m_Values[3];
  };  // Vector3




  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines an orientation as a quaternion rotation using the positive Z axis as the zero reference.
   * <BR>
   * Q = w + ix + jy + kz <BR>
   * w = c_1 * c_2 * c_3 - s_1 * s_2 * s_3 <BR>
   * x = s_1 * s_2 * c_3 + c_1 * c_2 * s_3 <BR>
   * y = s_1 * c_2 * c_3 + c_1 * s_2 * s_3 <BR>
   * z = c_1 * s_2 * c_3 - s_1 * c_2 * s_3 <BR>
   * where <BR>
   * c_1 = cos(theta/2) <BR>
   * c_2 = cos(phi/2) <BR>
   * c_3 = cos(psi/2) <BR>
   * s_1 = sin(theta/2) <BR>
   * s_2 = sin(phi/2) <BR>
   * s_3 = sin(psi/2) <BR>
   * and <BR>
   * theta is the angle of rotation about the Y axis measured from the Z axis. <BR>
   * phi is the angle of rotation about the Z axis measured from the X axis. <BR>
   * psi is the angle of rotation about the X axis measured from the Y axis. <BR>
   * (All angles are right-handed.)
   */
  class Quaternion
  {
  public:
    /**
     * Create a quaternion with default (x=0, y=0, z=0, w=1) values
     */
    inline Quaternion()
    {
      m_Values[0] = 0.0;
      m_Values[1] = 0.0;
      m_Values[2] = 0.0;
      m_Values[3] = 1.0;
    }

    /**
     * Create a quaternion using x, y, z, w values.
     * @param x
     * @param y
     * @param z
     * @param w
     */
    inline Quaternion(kt_double x, kt_double y, kt_double z, kt_double w)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
      m_Values[3] = w;
    }

    /**
     * Copy constructor
     */
    inline Quaternion(const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];
    }

  public:
    /**
     * Returns the X-value
     * @return Return the X-value of the quaternion
     */
    inline kt_double GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the X-value
     * @param x X-value of the quaternion
     */
    inline void SetX(kt_double x)
    {
      m_Values[0] = x;
    }

    /**
     * Returns the Y-value
     * @return Return the Y-value of the quaternion
     */
    inline kt_double GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the Y-value
     * @param y Y-value of the quaternion
     */
    inline void SetY(kt_double y)
    {
      m_Values[1] = y;
    }

    /**
     * Returns the Z-value
     * @return Return the Z-value of the quaternion
     */
    inline kt_double GetZ() const
    {
      return m_Values[2];
    }

    /**
     * Sets the Z-value
     * @param z Z-value of the quaternion
     */
    inline void SetZ(kt_double z)
    {
      m_Values[2] = z;
    }

    /**
     * Returns the W-value
     * @return Return the W-value of the quaternion
     */
    inline kt_double GetW() const
    {
      return m_Values[3];
    }

    /**
     * Sets the W-value
     * @param w W-value of the quaternion
     */
    inline void SetW(kt_double w)
    {
      m_Values[3] = w;
    }

    /**
     * Converts this quaternion into Euler angles
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
     * @param rYaw
     * @param rPitch
     * @param rRoll
     */
    void ToEulerAngles(kt_double& rYaw, kt_double& rPitch, kt_double& rRoll) const
    {
      kt_double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

      if (test > 0.499)
      {
        // singularity at north pole
        rYaw = 2 * atan2(m_Values[0], m_Values[3]);;
        rPitch = KT_PI_2;
        rRoll = 0;
      }
      else if (test < -0.499)
      {
        // singularity at south pole
        rYaw = -2 * atan2(m_Values[0], m_Values[3]);
        rPitch = -KT_PI_2;
        rRoll = 0;
      }
      else
      {
        kt_double sqx = m_Values[0] * m_Values[0];
        kt_double sqy = m_Values[1] * m_Values[1];
        kt_double sqz = m_Values[2] * m_Values[2];

        rYaw = atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2], 1 - 2 * sqy - 2 * sqz);
        rPitch = asin(2 * test);
        rRoll = atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2], 1 - 2 * sqx - 2 * sqz);
      }
    }

    /**
     * Set x,y,z,w values of the quaternion based on Euler angles.
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
     * @param yaw
     * @param pitch
     * @param roll
     */
    void FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll)
    {
      kt_double angle;

      angle = yaw * 0.5;
      kt_double cYaw = cos(angle);
      kt_double sYaw = sin(angle);

      angle = pitch * 0.5;
      kt_double cPitch = cos(angle);
      kt_double sPitch = sin(angle);

      angle = roll * 0.5;
      kt_double cRoll = cos(angle);
      kt_double sRoll = sin(angle);

      m_Values[0] = sYaw * sPitch * cRoll + cYaw * cPitch * sRoll;
      m_Values[1] = sYaw * cPitch * cRoll + cYaw * sPitch * sRoll;
      m_Values[2] = cYaw * sPitch * cRoll - sYaw * cPitch * sRoll;
      m_Values[3] = cYaw * cPitch * cRoll - sYaw * sPitch * sRoll;
    }

    /**
     * Assignment operator
     * @param rQuaternion
     */
    inline Quaternion& operator = (const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];

      return(*this);
    }

    /**
     * Equality operator returns true if the corresponding x, y, z, w values of each quaternion are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Quaternion& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] &&
              m_Values[1] == rOther.m_Values[1] &&
              m_Values[2] == rOther.m_Values[2] &&
              m_Values[3] == rOther.m_Values[3]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y, z, w values of each quaternion not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Quaternion& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] ||
              m_Values[1] != rOther.m_Values[1] ||
              m_Values[2] != rOther.m_Values[2] ||
              m_Values[3] != rOther.m_Values[3]);
    }

    /**
     * Write this quaternion onto output stream
     * @param rStream output stream
     * @param rQuaternion
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Quaternion& rQuaternion)
    {
      rStream << rQuaternion.m_Values[0] << " "
              << rQuaternion.m_Values[1] << " "
              << rQuaternion.m_Values[2] << " "
              << rQuaternion.m_Values[3];
      return rStream;
    }

  private:
    kt_double m_Values[4];
  };



  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Stores x, y, width and height that represents the location and size of a rectangle
   * (x, y) is at bottom left in mapper!
   */
  template<typename T>
  class Rectangle2
  {
  public:
    /**
     * Default constructor
     */
    Rectangle2()
    {
    }

    /**
     * Constructor initializing rectangle parameters
     * @param x x-coordinate of left edge of rectangle
     * @param y y-coordinate of bottom edge of rectangle
     * @param width width of rectangle
     * @param height height of rectangle
     */
    Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y)
      , m_Size(width, height)
    {
    }

    /**
     * Constructor initializing rectangle parameters
     * @param rPosition (x,y)-coordinate of rectangle
     * @param rSize Size of the rectangle
     */
    Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
      : m_Position(rPosition)
      , m_Size(rSize)
    {
    }

    /**
     * Copy constructor
     */
    Rectangle2(const Rectangle2& rOther)
      : m_Position(rOther.m_Position)
      , m_Size(rOther.m_Size)
    {
    }

  public:
    /**
     * Gets the x-coordinate of the left edge of this rectangle
     * @return the x-coordinate of the left edge of this rectangle
     */
    inline T GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate of the left edge of this rectangle
     * @param x the x-coordinate of the left edge of this rectangle
     */
    inline void SetX(T x)
    {
      m_Position.SetX(x);
    }

    /**
     * Gets the y-coordinate of the bottom edge of this rectangle
     * @return the y-coordinate of the bottom edge of this rectangle
     */
    inline T GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate of the bottom edge of this rectangle
     * @param y the y-coordinate of the bottom edge of this rectangle
     */
    inline void SetY(T y)
    {
      m_Position.SetY(y);
    }

    /**
     * Gets the width of this rectangle
     * @return the width of this rectangle
     */
    inline T GetWidth() const
    {
      return m_Size.GetWidth();
    }

    /**
     * Sets the width of this rectangle
     * @param width the width of this rectangle
     */
    inline void SetWidth(T width)
    {
      m_Size.SetWidth(width);
    }

    /**
     * Gets the height of this rectangle
     * @return the height of this rectangle
     */
    inline T GetHeight() const
    {
      return m_Size.GetHeight();
    }

    /**
     * Sets the height of this rectangle
     * @param height the height of this rectangle
     */
    inline void SetHeight(T height)
    {
      m_Size.SetHeight(height);
    }

    /**
     * Gets the position of this rectangle
     * @return the position of this rectangle
     */
    inline const Vector2<T>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position of this rectangle
     * @param rX x
     * @param rY y
     */
    inline void SetPosition(const T& rX, const T& rY)
    {
      m_Position = Vector2<T>(rX, rY);
    }

    /**
     * Sets the position of this rectangle
     * @param rPosition position
     */
    inline void SetPosition(const Vector2<T>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Gets the size of this rectangle
     * @return the size of this rectangle
     */
    inline const Size2<T>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets the size of this rectangle
     * @param rSize size
     */
    inline void SetSize(const Size2<T>& rSize)
    {
      m_Size = rSize;
    }

    /**
     * Gets the center of this rectangle
     * @return the center of this rectangle
     */
    inline const Vector2<T> GetCenter() const
    {
      return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5, m_Position.GetY() + m_Size.GetHeight() * 0.5);
    }

  public:
    /**
     * Assignment operator
     */
    Rectangle2& operator = (const Rectangle2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Size = rOther.m_Size;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Rectangle2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Rectangle2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
    }

  private:
    Vector2<T> m_Position;
    Size2<T> m_Size;
  };  // Rectangle2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Pose3;

  /**
   * Defines a position (x, y) in 2-dimensional space and heading.
   */
  class Pose2
  {
  public:
    /**
     * Default Constructor
     */
    Pose2()
      : m_Heading(0.0)
    {
    }

    /**
     * Constructor initializing pose parameters
     * @param rPosition position
     * @param heading heading
     **/
    Pose2(const Vector2<kt_double>& rPosition, kt_double heading)
      : m_Position(rPosition)
      , m_Heading(heading)
    {
    }

    /**
     * Constructor initializing pose parameters
     * @param x x-coordinate
     * @param y y-coordinate
     * @param heading heading
     **/
    Pose2(kt_double x, kt_double y, kt_double heading)
      : m_Position(x, y)
      , m_Heading(heading)
    {
    }

    /**
     * Constructs a Pose2 object from a Pose3.
     */
    Pose2(const Pose3& rPose);

    /**
     * Copy constructor
     */
    Pose2(const Pose2& rOther)
      : m_Position(rOther.m_Position)
      , m_Heading(rOther.m_Heading)
    {
    }

  public:
    /**
     * Returns the x-coordinate
     * @return the x-coordinate of the pose
     */
    inline kt_double GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate
     * @param x the x-coordinate of the pose
     */
    inline void SetX(kt_double x)
    {
      m_Position.SetX(x);
    }

    /**
     * Returns the y-coordinate
     * @return the y-coordinate of the pose
     */
    inline kt_double GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate
     * @param y the y-coordinate of the pose
     */
    inline void SetY(kt_double y)
    {
      m_Position.SetY(y);
    }

    /**
     * Returns the position
     * @return the position of the pose
     */
    inline const Vector2<kt_double>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position
     * @param rPosition of the pose
     */
    inline void SetPosition(const Vector2<kt_double>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Returns the heading of the pose (in radians)
     * @return the heading of the pose
     */
    inline kt_double GetHeading() const
    {
      return m_Heading;
    }

    /**
     * Sets the heading
     * @param heading of the pose
     */
    inline void SetHeading(kt_double heading)
    {
      m_Heading = heading;
    }

    /**
     * Return the squared distance between two Pose2
     * @return squared distance
     */
    inline kt_double SquaredDistance(const Pose2& rOther) const
    {
      return m_Position.SquaredDistance(rOther.m_Position);
    }

  public:
    /**
     * Assignment operator
     */
    inline Pose2& operator = (const Pose2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Heading = rOther.m_Heading;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Pose2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Pose2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
    }

    /**
     * In place Pose2 add.
     */
    inline void operator += (const Pose2& rOther)
    {
      m_Position += rOther.m_Position;
      m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
    }

    /**
     * Binary Pose2 add
     * @param rOther
     * @return Pose2 sum
     */
    inline Pose2 operator + (const Pose2& rOther) const
    {
      return Pose2(m_Position + rOther.m_Position, math::NormalizeAngle(m_Heading + rOther.m_Heading));
    }

    /**
     * Binary Pose2 subtract
     * @param rOther
     * @return Pose2 difference
     */
    inline Pose2 operator - (const Pose2& rOther) const
    {
      return Pose2(m_Position - rOther.m_Position, math::NormalizeAngle(m_Heading - rOther.m_Heading));
    }

    /**
     * Read pose from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Pose2& /*rPose*/)
    {
      // Implement me!!
      return rStream;
    }

    /**
     * Write this pose onto output stream
     * @param rStream output stream
     * @param rPose to read
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Pose2& rPose)
    {
      rStream << rPose.m_Position.GetX() << " " << rPose.m_Position.GetY() << " " << rPose.m_Heading;
      return rStream;
    }

  private:
    Vector2<kt_double> m_Position;

    kt_double m_Heading;
  };  // Pose2

  /**
   * Type declaration of Pose2 vector
   */
  typedef std::vector< Pose2 > Pose2Vector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a position and orientation in 3-dimensional space.
   * Karto uses a right-handed coordinate system with X, Y as the 2-D ground plane and X is forward and Y is left.
   * Values in Vector3 used to define position must have units of meters.
   * The value of angle when defining orientation in two dimensions must be in units of radians.
   * The definition of orientation in three dimensions uses quaternions.
   */
  class Pose3
  {
  public:
    /**
     * Default constructor
     */
    Pose3()
    {
    }

    /**
     * Create a new Pose3 object from the given position.
     * @param rPosition position vector in three space.
     */
    Pose3(const Vector3<kt_double>& rPosition)
      : m_Position(rPosition)
    {
    }

    /**
     * Create a new Pose3 object from the given position and orientation.
     * @param rPosition position vector in three space.
     * @param rOrientation quaternion orientation in three space.
     */
    Pose3(const Vector3<kt_double>& rPosition, const karto::Quaternion& rOrientation)
      : m_Position(rPosition)
      , m_Orientation(rOrientation)
    {
    }

    /**
     * Copy constructor
     */
    Pose3(const Pose3& rOther)
      : m_Position(rOther.m_Position)
      , m_Orientation(rOther.m_Orientation)
    {
    }

    /**
     * Constructs a Pose3 object from a Pose2.
     */
    Pose3(const Pose2& rPose)
    {
      m_Position = Vector3<kt_double>(rPose.GetX(), rPose.GetY(), 0.0);
      m_Orientation.FromEulerAngles(rPose.GetHeading(), 0.0, 0.0);
    }

  public:
    /**
     * Get the position of the pose as a 3D vector as const. Values have units of meters.
     * @return 3-dimensional position vector as const
     */
    inline const Vector3<kt_double>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Set the position of the pose as a 3D vector. Values have units of meters.
     * @return 3-dimensional position vector
     */
    inline void SetPosition(const Vector3<kt_double>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Get the orientation quaternion of the pose as const.
     * @return orientation quaternion as const
     */
    inline const Quaternion& GetOrientation() const
    {
      return m_Orientation;
    }

    /**
     * Get the orientation quaternion of the pose.
     * @return orientation quaternion
     */
    inline void SetOrientation(const Quaternion& rOrientation)
    {
      m_Orientation = rOrientation;
    }

    /**
     * Returns a string representation of this pose
     * @return string representation of this pose
     */
    inline std::string ToString()
    {
      std::stringstream converter;
      converter.precision(std::numeric_limits<double>::digits10);

      converter << GetPosition() << " " << GetOrientation();

      return converter.str();
    }

  public:
    /**
     * Assignment operator
     */
    inline Pose3& operator = (const Pose3& rOther)
    {
      m_Position = rOther.m_Position;
      m_Orientation = rOther.m_Orientation;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Pose3& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Orientation == rOther.m_Orientation);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Pose3& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Orientation != rOther.m_Orientation);
    }

    /**
     * Write Pose3 onto output stream
     * @param rStream output stream
     * @param rPose to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Pose3& rPose)
    {
      rStream << rPose.GetPosition() << ", " << rPose.GetOrientation();
      return rStream;
    }

    /**
     * Read Pose3 from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Pose3& /*rPose*/)
    {
      // Implement me!!
      return rStream;
    }

  private:
    Vector3<kt_double> m_Position;
    Quaternion m_Orientation;
  };  // Pose3



  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a Matrix 3 x 3 class.
   */
  class Matrix3
  {
  public:
    /**
     * Default constructor
     */
    Matrix3()
    {
      Clear();
    }

    /**
     * Copy constructor
     */
    inline Matrix3(const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
    }

  public:
    /**
     * Sets this matrix to identity matrix
     */
    void SetToIdentity()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));

      for (kt_int32s i = 0; i < 3; i++)
      {
        m_Matrix[i][i] = 1.0;
      }
    }

    /**
     * Sets this matrix to zero matrix
     */
    void Clear()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));
    }

    /**
     * Sets this matrix to be the rotation matrix of rotation around given axis
     * @param x x-coordinate of axis
     * @param y y-coordinate of axis
     * @param z z-coordinate of axis
     * @param radians amount of rotation
     */
    void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
    {
      kt_double cosRadians = cos(radians);
      kt_double sinRadians = sin(radians);
      kt_double oneMinusCos = 1.0 - cosRadians;

      kt_double xx = x * x;
      kt_double yy = y * y;
      kt_double zz = z * z;

      kt_double xyMCos = x * y * oneMinusCos;
      kt_double xzMCos = x * z * oneMinusCos;
      kt_double yzMCos = y * z * oneMinusCos;

      kt_double xSin = x * sinRadians;
      kt_double ySin = y * sinRadians;
      kt_double zSin = z * sinRadians;

      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
      m_Matrix[0][1] = xyMCos - zSin;
      m_Matrix[0][2] = xzMCos + ySin;

      m_Matrix[1][0] = xyMCos + zSin;
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
      m_Matrix[1][2] = yzMCos - xSin;

      m_Matrix[2][0] = xzMCos - ySin;
      m_Matrix[2][1] = yzMCos + xSin;
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
    }

    /**
     * Returns transposed version of this matrix
     * @return transposed matrix
     */
    Matrix3 Transpose() const
    {
      Matrix3 transpose;

      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          transpose.m_Matrix[row][col] = m_Matrix[col][row];
        }
      }

      return transpose;
    }

    /**
     * Returns the inverse of the matrix
     */
    Matrix3 Inverse() const
    {
      Matrix3 kInverse = *this;
      kt_bool haveInverse = InverseFast(kInverse, 1e-14);
      if (haveInverse == false)
      {
        assert(false);
      }
      return kInverse;
    }

    /**
     * Internal helper method for inverse matrix calculation
     * This code is lifted from the OgreMatrix3 class!!
     */
    kt_bool InverseFast(Matrix3& rkInverse, kt_double fTolerance = KT_TOLERANCE) const
    {
      // Invert a 3x3 using cofactors.  This is about 8 times faster than
      // the Numerical Recipes code which uses Gaussian elimination.
      rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
      rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
      rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
      rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
      rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
      rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
      rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
      rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
      rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];

      kt_double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] +
                       m_Matrix[0][1]*rkInverse.m_Matrix[1][0] +
                       m_Matrix[0][2]*rkInverse.m_Matrix[2][0];

      if (fabs(fDet) <= fTolerance)
      {
        return false;
      }

      kt_double fInvDet = 1.0/fDet;
      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          rkInverse.m_Matrix[row][col] *= fInvDet;
        }
      }

      return true;
    }

    /**
     * Returns a string representation of this matrix
     * @return string representation of this matrix
     */
    inline std::string ToString() const
    {
      std::stringstream converter;
      converter.precision(std::numeric_limits<double>::digits10);

      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          converter << m_Matrix[row][col] << " ";
        }
      }

      return converter.str();
    }

  public:
    /**
     * Assignment operator
     */
    inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
      return *this;
    }

    /**
     * Matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return reference to mat(r,c)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      return m_Matrix[row][column];
    }

    /**
     * Read-only matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return mat(r,c)
     */
    inline kt_double operator()(kt_int32u row, kt_int32u column) const
    {
      return m_Matrix[row][column];
    }

    /**
     * Binary Matrix3 multiplication.
     * @param rOther
     * @return Matrix3 product
     */
    Matrix3 operator * (const Matrix3& rOther) const
    {
      Matrix3 product;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] +
                                       m_Matrix[row][1]*rOther.m_Matrix[1][col] +
                                       m_Matrix[row][2]*rOther.m_Matrix[2][col];
        }
      }

      return product;
    }

    /**
     * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
     * @param rPose2
     * @return Pose2 product
     */
    inline Pose2 operator * (const Pose2& rPose2) const
    {
      Pose2 pose2;

      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
                 rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
                 rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
                       rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

      return pose2;
    }

    /**
     * In place Matrix3 add.
     * @param rkMatrix
     */
    inline void operator += (const Matrix3& rkMatrix)
    {
      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
        }
      }
    }

    /**
     * Write Matrix3 onto output stream
     * @param rStream output stream
     * @param rMatrix to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Matrix3& rMatrix)
    {
      rStream << rMatrix.ToString();
      return rStream;
    }

  private:
    kt_double m_Matrix[3][3];
  };  // Matrix3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a general Matrix class.
   */
  class Matrix
  {
  public:
    /**
     * Constructs a matrix of size rows x columns
     */
    Matrix(kt_int32u rows, kt_int32u columns)
      : m_Rows(rows)
      , m_Columns(columns)
      , m_pData(NULL)
    {
      Allocate();

      Clear();
    }

    /**
     * Destructor
     */
    virtual ~Matrix()
    {
      delete [] m_pData;
    }

  public:
    /**
     * Set all entries to 0
     */
    void Clear()
    {
      if (m_pData != NULL)
      {
        memset(m_pData, 0, sizeof(kt_double) * m_Rows * m_Columns);
      }
    }

    /**
     * Gets the number of rows of the matrix
     * @return nubmer of rows
     */
    inline kt_int32u GetRows() const
    {
      return m_Rows;
    }

    /**
     * Gets the number of columns of the matrix
     * @return nubmer of columns
     */
    inline kt_int32u GetColumns() const
    {
      return m_Columns;
    }

    /**
     * Returns a reference to the entry at (row,column)
     * @param row
     * @param column
     * @return reference to entry at (row,column)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

    /**
     * Returns a const reference to the entry at (row,column)
     * @param row
     * @param column
     * @return const reference to entry at (row,column)
     */
    inline const kt_double& operator()(kt_int32u row, kt_int32u column) const
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

  private:
    /**
     * Allocate space for the matrix
     */
    void Allocate()
    {
      try
      {
        if (m_pData != NULL)
        {
          delete[] m_pData;
        }

        m_pData = new kt_double[m_Rows * m_Columns];
      }
      catch (const std::bad_alloc& ex)
      {
        throw Exception("Matrix allocation error");
      }

      if (m_pData == NULL)
      {
        throw Exception("Matrix allocation error");
      }
    }

    /**
     * Checks if (row,column) is a valid entry into the matrix
     * @param row
     * @param column
     */
    inline void RangeCheck(kt_int32u row, kt_int32u column) const
    {
      if (math::IsUpTo(row, m_Rows) == false)
      {
        throw Exception("Matrix - RangeCheck ERROR!!!!");
      }

      if (math::IsUpTo(column, m_Columns) == false)
      {
        throw Exception("Matrix - RangeCheck ERROR!!!!");
      }
    }

  private:
    kt_int32u m_Rows;
    kt_int32u m_Columns;

    kt_double* m_pData;
  };  // Matrix




  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a bounding box in 2-dimensional real space.
   */
  class BoundingBox2
  {
  public:
    /*
     * Default constructor
     */
    BoundingBox2()
      : m_Minimum(999999999999999999.99999, 999999999999999999.99999)
      , m_Maximum(-999999999999999999.99999, -999999999999999999.99999)
    {
    }

  public:
    /**
     * Get bounding box minimum
     */
    inline const Vector2<kt_double>& GetMinimum() const
    {
      return m_Minimum;
    }

    /**
     * Set bounding box minimum
     */
    inline void SetMinimum(const Vector2<kt_double>& mMinimum)
    {
      m_Minimum = mMinimum;
    }

    /**
     * Get bounding box maximum
     */
    inline const Vector2<kt_double>& GetMaximum() const
    {
      return m_Maximum;
    }

    /**
     * Set bounding box maximum
     */
    inline void SetMaximum(const Vector2<kt_double>& rMaximum)
    {
      m_Maximum = rMaximum;
    }

    /**
     * Get the size of the bounding box
     */
    inline Size2<kt_double> GetSize() const
    {
      Vector2<kt_double> size = m_Maximum - m_Minimum;

      return Size2<kt_double>(size.GetX(), size.GetY());
    }

    /**
     * Add vector to bounding box
     */
    inline void Add(const Vector2<kt_double>& rPoint)
    {
      m_Minimum.MakeFloor(rPoint);
      m_Maximum.MakeCeil(rPoint);
    }

    /**
     * Add other bounding box to bounding box
     */
    inline void Add(const BoundingBox2& rBoundingBox)
    {
      Add(rBoundingBox.GetMinimum());
      Add(rBoundingBox.GetMaximum());
    }

    /**
     * Whether the given point is in the bounds of this box
     * @param rPoint
     * @return in bounds?
     */
    inline kt_bool IsInBounds(const Vector2<kt_double>& rPoint) const
    {
      return (math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
              math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
    }

  private:
    Vector2<kt_double> m_Minimum;
    Vector2<kt_double> m_Maximum;
  };  // BoundingBox2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Implementation of a Pose2 transform
   */
  class Transform
  {
  public:
    /**
     * Constructs a transformation from the origin to the given pose
     * @param rPose pose
     */
    Transform(const Pose2& rPose)
    {
      SetTransform(Pose2(), rPose);
    }

    /**
     * Constructs a transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    Transform(const Pose2& rPose1, const Pose2& rPose2)
    {
      SetTransform(rPose1, rPose2);
    }

  public:
    /**
     * Transforms the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 TransformPose(const Pose2& rSourcePose)
    {
      Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

      return Pose2(newPosition.GetPosition(), angle);
    }

    /**
     * Inverse transformation of the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 InverseTransformPose(const Pose2& rSourcePose)
    {
      Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

      // components of transform
      return Pose2(newPosition.GetPosition(), angle);
    }

  private:
    /**
     * Sets this to be the transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    void SetTransform(const Pose2& rPose1, const Pose2& rPose2)
    {
      if (rPose1 == rPose2)
      {
        m_Rotation.SetToIdentity();
        m_InverseRotation.SetToIdentity();
        m_Transform = Pose2();
        return;
      }

      // heading transformation
      m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
      m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

      // position transformation
      Pose2 newPosition;
      if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
      {
        newPosition = rPose2 - m_Rotation * rPose1;
      }
      else
      {
        newPosition = rPose2;
      }

      m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
    }

  private:
    // pose transformation
    Pose2 m_Transform;

    Matrix3 m_Rotation;
    Matrix3 m_InverseRotation;
  };  // Transform




}  // namespace karto

#endif  // OPEN_KARTO_MATH_H
