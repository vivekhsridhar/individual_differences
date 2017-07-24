//
//  vector2D.cpp
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include "vector2D.h"

//**************************************************************************************************
//**	CONSTRUCTORS OF CLASS 'VECTOR2D'	********************************************************
//**************************************************************************************************

CVec2D::CVec2D(void)
{
    x = 0.0f; y = 0.0f;
}

CVec2D::~CVec2D(void)
{
}

CVec2D::CVec2D(double x1, double y1)
{
    x = x1; y = y1;
}

CVec2D::CVec2D(CVec2D& other_vector)
{
    x = other_vector.x; y = other_vector.y;
}

//**************************************************************************************************
//**	OTHER MEMBER FUNCTIONS	********************************************************************
//**************************************************************************************************

// add vectors
CVec2D CVec2D::operator+(const CVec2D& vec)
{
    CVec2D result;
    result.x = x + vec.x;
    result.y = y + vec.y;
    return result;
}

CVec2D CVec2D::operator+=(const CVec2D& vec)
{
    x += vec.x;
    y += vec.y;
    return *this;
}

// subtract vectors
CVec2D CVec2D::operator-(const CVec2D& vec)
{
    CVec2D result;
    result.x = x - vec.x;
    result.y = y - vec.y;
    return result;
}

CVec2D CVec2D::operator-=(const CVec2D& vec)
{
    x -= vec.x;
    y -= vec.y;
    return *this;
}

// negative vector
CVec2D CVec2D::operator-()
{
    CVec2D result;
    result.x = -x;
    result.y = -y;
    return result;
}

// equate to vector
CVec2D CVec2D::operator=(const CVec2D& vec)
{
    x = vec.x;
    y = vec.y;
    return *this;
}

// multiply vectors - elementwise
CVec2D CVec2D::operator*(const CVec2D& vec)
{
    CVec2D result;
    result.x = x * vec.x;
    result.y = y * vec.y;
    return result;
}

CVec2D CVec2D::operator*=(const CVec2D& vec)
{
    x *=vec.x;
    y *=vec.y;
    return *this;
}

// multiply vector with scalar
CVec2D CVec2D::operator*(double val)
{
    CVec2D result;
    result.x = x * val;
    result.y = y * val;
    return result;
}

CVec2D CVec2D::operator*=(double val)
{
    x *= val;
    y *= val;
    return *this;
}

// divide vectors - elementwise
CVec2D CVec2D::operator/(const CVec2D& vec)
{
    CVec2D result;
    result.x = x / vec.x;
    result.y = y / vec.y;
    return result;
}

// divide vector by scalar
CVec2D CVec2D::operator/(double val)
{
    CVec2D result;
    result.x = x / val;
    result.y = y / val;
    return result;
}

CVec2D CVec2D::operator/=(double val)
{
    x /= val;
    y /= val;
    return *this;
}

// normalise
CVec2D CVec2D::normalise()
{
    CVec2D result;
    double dist = this->length();
    
    if(dist != 0.0)
    {
        result.x = x / dist;
        result.y = y / dist;
        return result;
    }
    else
    {
        // make no change
        return *this;
    }
}

// dot product
double CVec2D::dot(const CVec2D& vec)
{
    double result;
    result = x*vec.x + y*vec.y;
    return result;
}

// cross product
double CVec2D::cross(const CVec2D& vec)
{
    double result;
    result = x*vec.y - vec.x*y;
    
    return result;
}

// rotate vector
void CVec2D::rotate(double degrees)
{
    double temp_x = x;
    double temp_y = y;
    temp_x = ( (x * cos(degrees * PiOver180)) - (y * sin(degrees * PiOver180)) );
    temp_y = ( (x * sin(degrees * PiOver180)) + (y * cos(degrees * PiOver180)) );
    x = temp_x;
    y = temp_y;
}

// length
double CVec2D::length()
{
    //	return (sqrt(Num.SqrFP(x) + Num.SqrFP(y)));	// too slow
    return sqrt((x*x) + (y*y));
}

// squared distance to another vector
double CVec2D::distanceTo(const CVec2D& to_point)
{
    double dist_x = to_point.x - x;
    double dist_y = to_point.y - y;
    
    return (dist_x * dist_x + dist_y * dist_y);
}

// angle from x-axis (east) to vector - counterclockwise
double CVec2D::polarAngle()
{
    if((x == 0.0f) && (y == 0.0f)) return -1.0f;

    if(x == 0.0f) return ((y > 0.0f) ? 90.0f : 270.0f);
    
    double theta = atan(y / x);	// in radians
    theta *= PiUnder180;
    
    if(x > 0.0)	return ((y >= 0) ? theta : 360.0 + theta);  // quadrants 1 and 4
    else return (180.0 + theta);  // quadrants 2 and 3
}

//  angle from vector to y-axis (north) - counterclockwise
double CVec2D::polarAngleZeroNorth()
{
    if((x == 0.0f) && (y == 0.0f)) return -1.0f;
    
    if(x == 0.0f) return ((y > 0.0f) ? 90.0f : 270.0f);
    
    double theta = atan(y / x);	// in radians
    theta *= PiUnder180;
    
    if(x > 0.0) return (90.0 - theta);  // quadrants 1 and 4
    else return (270.0 - theta);  // quadrants 2 and 3
}

// smallest angle to another vector
double CVec2D::smallestAngleTo(CVec2D other_vector)
{
    // this function calculates the smaller of the two angles between two unit vectors that begin at the origin
    CVec2D vec = *this;
    vec = vec.normalise();	// the speed is unimportant so the vector can be normalised
    
    double dot_product = vec.dot(other_vector);
    double val = (acos(dot_product));
    return (val/PiOver180);
}
