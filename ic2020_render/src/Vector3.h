/************************************************************************
 * 
 * This source code is part of the IC2020 SLAM System
 * 
 * Vector3 Copyright (C) 2008
 * Sean Anderson
 * 
 * IC2020 is licenced under the Creative Commons License
 * Attribution-NonCommercial-ShareAlike 2.5 Canada
 * (CC BY-NC-SA 2.5).
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 ************************************************************************/

#ifndef VECTOR3_H_
#define VECTOR3_H_

class Vector3
{
public:
    Vector3();
    Vector3(float x, float y, float z);
    ~Vector3();
    Vector3(const Vector3 &vec);

    Vector3 operator+(const Vector3 &vec) const;
    Vector3 operator-(const Vector3 &vec) const;
    Vector3 operator*(const float k) const;

    void    Normalize();
    float   Dot(const Vector3 &vec) const;
    Vector3 Cross(const Vector3 &vec) const;
    float   Length();

    bool IsEqual(const Vector3 &vec) const;

    float   x;
    float   y;
    float   z;
};

#endif /* VECTOR3_H_ */
