//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_CIRCLE_H
#define NAVIGATION_CIRCLE_H

class Circle {
public:
    double radius;
    Vector2D point;
    Circle(){
        radius = 0.0;
        point = Vector2D();
    }
    Circle(Vector2D point_, double radius_){
        radius = radius_;
        point = point_;
    }
    vector<Vector2D> toPoints(u_int nPoints, bool inside = true){
        vector<Vector2D> circlePoints;
        if( inside ){
            Vector2D p(radius,0);
            double angleIncrement = (M_PI*2)/nPoints;
            for(u_int i = 0; i < nPoints; i++){
                circlePoints.emplace_back(p+point);
                p.transformThis(0, 0, angleIncrement);
            }
        }else{
            //TODO outside circle points
        }
        return circlePoints;
    }
    bool circleContainsPoint(const Vector2D& p){
        double distance = point.distance(p);
        return (distance <= radius);
    }
};


#endif //NAVIGATION_CIRCLE_H
