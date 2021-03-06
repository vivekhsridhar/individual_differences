//
//  vector2D.h
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 14/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef vector2D_h
#define vector2D_h


#include <cmath>
#include <stdlib.h>

const double Pi = 3.1415927;
const double PiOver180 = 1.74532925199433E-002;
const double PiUnder180 = 5.72957795130823E+001;

//**************************************************************************************************
//**	CLASS DEFINITION	************************************************************************
//**************************************************************************************************

class CVec2D
{
public:
    CVec2D(void);                           // default constructor
    ~CVec2D(void);                          // class destructor
    
    CVec2D(double x1, double x2);           // alternative constructor
    CVec2D(CVec2D&);                        // copy constructor
    double x, y;
    
    CVec2D operator+(const CVec2D&);
    CVec2D operator+=(const CVec2D&);
    CVec2D operator-(const CVec2D&);
    CVec2D operator-=(const CVec2D&);
    CVec2D operator-();
    CVec2D operator=(const CVec2D&);
    CVec2D operator*(const CVec2D&);
    CVec2D operator*=(const CVec2D&);
    CVec2D operator*(double);
    CVec2D operator*=(double);
    CVec2D operator/(const CVec2D&);
    CVec2D operator/(double);
    CVec2D operator/=(double);
    
    CVec2D normalise();
    double dot(const CVec2D&);
    double cross(const CVec2D&);
    void rotate(double degrees);
    double length();
    double distanceTo(const CVec2D&);
    inline void Clear() { x = 0.0; y = 0.0; }
    double polarAngle();
    double polarAngleZeroNorth();
    double smallestAngleTo(CVec2D);
};

#endif /* vector2D_h */
