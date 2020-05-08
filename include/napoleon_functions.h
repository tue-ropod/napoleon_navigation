#ifndef NAP_FUN_H
#define NAP_FUN_H

using namespace std;

#include <string>
#include <array>
#include "napoleon_config.h"
#include <vector>


static constexpr double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static constexpr double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

void printstringvec(vector<string> vec);
Point getPoint(PointID point_with_id);
Point rotate_point(Point c, double angle, Point p);
Point coordGlobalToRopod(Point gp, Point rc, double ra);
Point coordGlobalToRopod(PointID gp, Point rc, double ra);
double getSteering(Point local_wallpoint_front, Point local_wallpoint_rear, double tube_width);
double getSteeringTurn(Point local_pivot, bool dir_cw, double tubewidth, Point local_wallpoint_front, Point local_wallpoint_rear);
double steerAroundPoint(Point local_pivot, bool dir_cw);
double dist2(Point v, Point w);
double distToSegmentSquared(Point p, Point v, Point w);
double distToEndSegmentSquared(Point p, Point v, Point w);
double distToEndSegmentSquared(Point p, PointID v, PointID w);
bool isPointOnLeftSide(string p_rearID, string p_frontID, vector<PointID> pointlist, Point point, double max_dist);
double perpDistToSegment(Point p, PointID v, PointID w);
double perpDistToSegment(Point p, Point v, Point w);
double distToSegment(Point p, PointID v, PointID w);
double distToSegment(Point p, Point v, Point w);
double distToLine(Point p, PointID v, PointID w);
double distToLine(Point p, Point v, Point w);
double distToEllipse(Point X, PointID C, PointID semi_major, PointID semi_minor);
bool do_lines_intersect(Point p0, Point p1, Point p2, Point p3);
bool does_line_intersect_shape(Point p0, Point p1, AreaQuad shape);
bool do_shapes_overlap(Point obj1p0, Point obj1p1, Point obj1p2, Point obj1p3, Point obj2p0, Point obj2p1, Point obj2p2, Point obj2p3);
bool do_shapes_overlap(double shape1[][2], double shape2[][2]);
bool does_line_intersect_shape(Point p0, Point p1, double shape[][2]);
bool doesShapeCollideWithCircle(double shape[][2], Point c, double r);
AreaQuad generateEntry(int hallwayID, int interID, double e_length, vector<AreaQuadID> arealist, vector<PointID> pointlist);
double getAngleBetweenHallways(vector<string> taskhallway1, vector<string> taskhallway2, vector<PointID> pointlist);
AreaQuadID getAreaByID(int wantedID, vector<AreaQuadID> arealist);
vector<string> getWallPointsTowardsB(AreaQuadID A, AreaQuadID B);
vector<string> getWallPointsAwayFromB(AreaQuadID A, AreaQuadID B);
vector<string> getCommonPoints(AreaQuadID A, AreaQuadID B);
PointID getPointByID(string wantedID, vector<PointID> pointlist);
vector<string> getPointsForTurning(AreaQuadID OBJ1, AreaQuadID OBJ2, AreaQuadID OBJ3, vector<string> OBJ1TASK);
vector<string> getWalls(int id_OBJ1, int id_OBJ2, int id_OBJ3, vector<AreaQuadID> arealist);

double getSteering(Point local_wallpoint_front, Point local_wallpoint_rear, double tubewidth, double carrot_length, double feeler_size);
double getSteeringTurn(Point local_pivot, bool dir_cw, double tubewidth, Point local_wallpoint_front, Point local_wallpoint_rear, double carrot_length, double feeler_size);
double getSteeringTurnSharp(Point ropodpos, double ropod_angle, bool dir_cw, std::vector<string> task, vector<PointID> pointlist, double carrot_length, double feeler_size_steering);

double wrapToPi(double angle);
double modf(double x, double y);
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
string get_date(void);

// State of ropod
enum{
    CRUISING = 1,
    ENTRY_BEFORE_TURN_ON_INTERSECTION,
    ACCELERATE_ON_INTERSECTION,
    ALIGN_AXIS_AT_INTERSECTION, //  (& slow down)
    TURNING,
    ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION,
    GOING_STRAIGHT_ON_INTERSECTION,
    TIGHT_OVERTAKE,     // (follow left wall)
    SPACIOUS_OVERTAKE,   // (shift right wall to left virtually)
    DEPARTURE,
    EXIT_TURN_RIGHT
};

#endif
