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
        v.x = this->x + other.x;
        v.y = this->y + other.y;
        return v;
    }
    Vector2D operator-= (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = this->x - other.x;
        v.y = this->y - other.y;
        return v;
    }
    Vector2D operator+ (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = this->x + other.x;
        v.y = this->y + other.y;
        return v;
    }
    Vector2D operator- (const Vector2D& other)const{
        Vector2D v(0,0);
        v.x = this->x - other.x;
        v.y = this->y - other.y;
        return v;
    }

    Vector2D operator/ (double s)const{
        Vector2D v(0,0);
        v.x = this->x/s;
        v.y = this->y/s;
        return v;
    }

    Vector2D operator* (double s)const{
        Vector2D v(0,0);
        v.x = this->x*s;
        v.y = this->y*s;
        return v;
    }
    bool operator== (const Vector2D& other)const{
        return (this->x == other.x && this->y == other.y);
    }
    double distance(const Vector2D &other)const{
        return sqrt(pow((this->x-other.x),2) + pow((this->y-other.y),2));
    }
    double length()const{
        Vector2D v(this->x,this->y);
        return sqrt(v.dot(v));
    }
    double angle()const{
        return atan2(this->y, this->x);
    }
    double dot(const Vector2D &other)const{
        return ((this->x * other.x) + (this->y * other.y));
    }
    void unitThis(){
        Vector2D v(this->x,this->y);
        if(v.length() != 0){
            v = v/v.length();
        }else{
            v.x = 0;
            v.y = 0;
        }
        this->x = v.x;
        this->y = v.y;
    }
    Vector2D unit()const{
        Vector2D v(this->x,this->y);
        if(v.length() != 0){
            v = v/v.length();
        }else{
            v.x = 0;
            v.y = 0;
        }
        return v;
    }
    void transformThis(double x_, double y_, double a_){
        double xtemp = this->x;
        double ytemp = this->y;
        this->x = xtemp*cos(a_) - ytemp*sin(a_) + x_;
        this->y = xtemp*sin(a_) + ytemp*cos(a_) + y_;
    }
    Vector2D transform(double x_, double y_, double a_)const{
        Vector2D v;
        v.x = this->x*cos(a_) - this->y*sin(a_) + x_;
        v.y = this->x*sin(a_) + this->y*cos(a_) + y_;
        return v;
    }
    void scaleThis(double x_, double y_){
        double xtemp = this->x;
        double ytemp = this->y;
        this->x = xtemp * x_;
        this->y = ytemp * y_;
    }
    Vector2D scale(double x_, double y_)const{
        Vector2D v;
        v.x = this->x * x_;
        v.y = this->y * y_;
        return v;
    }
};

inline void smallestAngle(double& angle){
    angle = fmod(angle + M_PI, M_PI*2);
    if (angle < 0) angle += M_PI*2;
    angle -= M_PI;
}

#endif //NAVIGATION_VECTOR2D_H

