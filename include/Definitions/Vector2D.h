//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_VECTOR2D_H
#define NAVIGATION_VECTOR2D_H

#include <cmath>

using namespace std;

struct Vector2D{
    double x;
    double y;
    Vector2D(double x_, double y_){
        x = x_;
        y = y_;
    }
    Vector2D(){
        x = 0;
        y = 0;
    }
    Vector2D operator+= (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = x + other.x;
        v.y = y + other.y;
        return v;
    }
    Vector2D operator-= (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = x - other.x;
        v.y = y - other.y;
        return v;
    }
    Vector2D operator+ (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = x + other.x;
        v.y = y + other.y;
        return v;
    }
    Vector2D operator- (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = x - other.x;
        v.y = y - other.y;
        return v;
    }

    Vector2D operator/ (double s)const{
        Vector2D v(0,0);
        v.x = x/s;
        v.y = y/s;
        return v;
    }

    Vector2D operator* (double s)const{
        Vector2D v(0,0);
        v.x = x*s;
        v.y = y*s;
        return v;
    }
    bool operator== (const Vector2D& other)const{
        return (x == other.x && y == other.y);
    }
    double distance(const Vector2D &other)const{
        return sqrt(pow((x-other.x),2) + pow((y-other.y),2));
    }
    double length()const{
        Vector2D v(x,y);
        return sqrt(v.dot(v));
    }
    double angle()const{
        return atan2(y, x);
    }
    double dot(const Vector2D &other)const{
        return ((x * other.x) + (y * other.y));
    }
    void unitThis(){
        Vector2D v(x,y);
        if(v.length() != 0){
            v = v/v.length();
        }else{
            v.x = 0;
            v.y = 0;
        }
        x = v.x;
        y = v.y;
    }
    Vector2D unit()const{
        Vector2D v(x,y);
        if(v.length() != 0){
            v = v/v.length();
        }else{
            v.x = 0;
            v.y = 0;
        }
        return v;
    }
    void transformThis(double x_, double y_, double a_){
        double xtemp = x;
        double ytemp = y;
        this->x = xtemp*cos(a_) - ytemp*sin(a_) + x_;
        this->y = xtemp*sin(a_) + ytemp*cos(a_) + y_;
    }
    Vector2D transform(double x_, double y_, double a_)const{
        Vector2D v;
        v.x = x*cos(a_) - y*sin(a_) + x_;
        v.y = x*sin(a_) + y*cos(a_) + y_;
        return v;
    }
    void scaleThis(double x_, double y_){
        double xtemp = x;
        double ytemp = y;
        this->x = xtemp * x_;
        this->y = ytemp * y_;
    }
    Vector2D scale(double x_, double y_)const{
        Vector2D v;
        v.x = x * x_;
        v.y = y * y_;
        return v;
    }
    bool valid()const{
        return !(isnan(x) || isnan(y) || isinf(x) || isinf(y));
    }
};

inline void smallestAngle(double& angle){
    angle = fmod(angle + M_PI, M_PI*2);
    if (angle < 0) angle += M_PI*2;
    angle -= M_PI;
}

#endif //NAVIGATION_VECTOR2D_H
