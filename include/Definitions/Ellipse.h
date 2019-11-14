//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_ELLIPSE_H
#define NAVIGATION_ELLIPSE_H

class Ellipse {
public:
    double a, b, angle;
    Vector2D point;
    Ellipse(){
        a = 0.0, b = 0.0;
        point = Vector2D();
    }
    Ellipse(Vector2D point_, double a_, double b_, double angle_){
        a = a_;
        b = b_;
        angle = angle_;
        point = point_;
    }
    vector<Vector2D> toPoints(u_int nPoints, bool inside = true){
        vector<Vector2D> points;
        if( inside ){
            Vector2D p;
            double angleIncrement = (M_PI*2)/nPoints;
            for(u_int i = 0; i < nPoints; i++){
                p = Vector2D(a*cos(angleIncrement*i), b*sin(angleIncrement*i));
                p.transformThis(0,0,angle);
                points.emplace_back(p+point);
            }
        }else{
            //TODO outside points
        }
        return points;
    }
};


#endif //NAVIGATION_ELLIPSE_H
