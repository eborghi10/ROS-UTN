#pragma once

class Angle {
public:
    Angle(double angle);
    inline double GetAngle();
    inline double ToDegrees();
    inline void NormalizeAngle();
    inline void NormalizeAnglePositive();
    Angle operator-(Angle angle);
    void operator-=(Angle angle);
    Angle operator+(Angle angle);
    void operator+=(Angle angle);
    void operator=(double angle);
    bool operator<(double arg);
    bool operator>(double arg);
private:
    double angle;
};

Angle::Angle(double angle)
{
    this->angle = angle;
}

inline double Angle::GetAngle() 
{
    return this->angle;
}

inline double Angle::ToDegrees()
{
    return this->angle * 180.0 / PI;
}

inline void Angle::NormalizeAnglePositive()
{
    this->angle = fmod(fmod(this->angle, TWO_PI) + TWO_PI, TWO_PI);
}

inline void Angle::NormalizeAngle()
{
    this->NormalizeAnglePositive();
    if (this->angle > PI) this->angle -= TWO_PI;
}

Angle Angle::operator-(Angle angle)
{
    return this->angle - angle.GetAngle();
}

void Angle::operator-=(Angle angle)
{
    this->angle -= angle.GetAngle();
}

Angle Angle::operator+(Angle angle)
{
    return this->angle + angle.GetAngle();
}

void Angle::operator+=(Angle angle)
{
    this->angle += angle.GetAngle();
}

void Angle::operator=(double angle)
{
  this->angle = angle;
}

bool Angle::operator<(double arg)
{
    return this->angle < arg;
}

bool Angle::operator>(double arg)
{
    return this->angle > arg;
}

