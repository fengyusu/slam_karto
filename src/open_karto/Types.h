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

#ifndef OPEN_KARTO_TYPES_H
#define OPEN_KARTO_TYPES_H





/**
 * Karto type definition
 * Ensures platform independent types for windows, linux and mac
 */
#if defined(_MSC_VER)

  typedef signed __int8 kt_int8s;
  typedef unsigned __int8 kt_int8u;

  typedef signed __int16 kt_int16s;
  typedef unsigned __int16 kt_int16u;

  typedef signed __int32 kt_int32s;
  typedef unsigned __int32 kt_int32u;

  typedef signed __int64 kt_int64s;
  typedef unsigned __int64 kt_int64u;

#else

  #include <stdint.h>

  typedef int8_t kt_int8s;
  typedef uint8_t kt_int8u;

  typedef int16_t kt_int16s;
  typedef uint16_t kt_int16u;

  typedef int32_t kt_int32s;
  typedef uint32_t kt_int32u;

#if defined(__LP64__)
  typedef signed long kt_int64s;
  typedef unsigned long kt_int64u;
#else
  typedef signed long long kt_int64s;
  typedef unsigned long long kt_int64u;
#endif

#endif

typedef bool kt_bool;
typedef char kt_char;
typedef float kt_float;
typedef double kt_double;

#define KARTO_Object(name) \
  virtual const char* GetClassName() const { return #name; } \
  virtual kt_objecttype GetObjectType() const { return ObjectType_##name; }

typedef kt_int32u kt_objecttype;

const kt_objecttype ObjectType_None                         = 0x00000000;
const kt_objecttype ObjectType_Sensor                       = 0x00001000;
const kt_objecttype ObjectType_SensorData                   = 0x00002000;
const kt_objecttype ObjectType_CustomData                   = 0x00004000;
const kt_objecttype ObjectType_Misc                         = 0x10000000;

const kt_objecttype ObjectType_Drive                        = ObjectType_Sensor | 0x01;
const kt_objecttype ObjectType_LaserRangeFinder             = ObjectType_Sensor | 0x02;
const kt_objecttype ObjectType_Camera                       = ObjectType_Sensor | 0x04;

const kt_objecttype ObjectType_DrivePose                    = ObjectType_SensorData | 0x01;
const kt_objecttype ObjectType_LaserRangeScan               = ObjectType_SensorData | 0x02;
const kt_objecttype ObjectType_LocalizedRangeScan           = ObjectType_SensorData | 0x04;
const kt_objecttype ObjectType_CameraImage                  = ObjectType_SensorData | 0x08;
const kt_objecttype ObjectType_LocalizedRangeScanWithPoints = ObjectType_SensorData | 0x16;

const kt_objecttype ObjectType_Header                       = ObjectType_Misc | 0x01;
const kt_objecttype ObjectType_Parameters                   = ObjectType_Misc | 0x02;
const kt_objecttype ObjectType_DatasetInfo                  = ObjectType_Misc | 0x04;
const kt_objecttype ObjectType_Module                       = ObjectType_Misc | 0x08;

namespace karto
{

} //end of namespace karto

#endif // OPEN_KARTO_TYPES_H
