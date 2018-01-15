//
//  Collision.h
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/08.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#ifndef Collision_h
#define Collision_h

#include "Primitive.h"

namespace myTools {
    
    //TODO: どこか適切なファイルに移動させること
    template<typename ty>
    void Clamp(ty& a, const ty min, const ty max){
        if( a < min){
            a = min;
        }
        if( a > max){
            a = max;
        }
    }
    
    bool SupPointLineColl(const Point& point, const Line& line, float& t);
    bool SupLineLineColl(const Line& line1, const Line& line2, float& t1, float& t2, Vector3& pos);
    bool SupPlaneLineColl(const PlaneCollision& plane, const Line& line, float& t, Vector3& pos);
    bool SupSphereLineColl(const SphereCollision& sphere, const Line& line,
                           float& t1, float& t2, Vector3& pos1, Vector3& pos2);
    float SupPointLineDist(const Point& point, const Line& line, Point& pos, float& t);
    float SupPointLineDistSq(const Point& point, const Line& line, Point& pos, float& t);

    float SupPointSegmentDist(const Point& point, const Segment& segment, float& t, Vector3& pos);
    float SupPointSegmentDistSq(const Point& point, const Segment& segment, float& t, Vector3& pos);
    float SupLineLineDist(const Line& line1, const Line& line2, float& t1, float& t2, Point& p1, Point& p2);
    float SupLineLineDistSq(const Line& line1, const Line& line2, float& t1, float& t2, Point& p1, Point& p2);
    float SupPlaneSegmentDist(const PlaneCollision& plane, const Segment& segment, float& t, Point& pos);
    
    float SupSegmentSegmentDist(const Segment& lhs, const Segment& rhs, float& t1, float& t2, Vector3& pos1, Vector3& pos2);
    
    PlaneCollision CastToPlaneCollision(const Vector3& v1, const Vector3& v2, const Vector3& v3);
    PlaneCollision CastToPlaneCollision(const PolygonCollision& polygon);
    PlaneCollision CastToPlaneCollision(const SquareCollision& polygon);

    bool IsFront(const PlaneCollision& plane, const Point& point);
    bool IsSharp(const Vector3& v1, const Vector3& v2, const Vector3& v3);
    bool IsInside(const Segment& segment, const Vector3& p);

    //Point and Point
    float Distance(const Point& p1, const Point& p2);
    
    //Point and LineCollision Distance
    float Distance(const Point& p, const Line& l);
    float Distance(const Line& l, const Point& p);
    
    float DistanceSq(const Point& p, const Line& l);
    float DistanceSq(const Line& l, const Point& p);
    
    //Point and Segment Distance
    float Distance(const Point& p, const Segment& s);
    float Distance(const Segment& s, const Point& p);
    
    float DistanceSq(const Point& point, const Segment& segment);
    float DistanceSq(const Segment& segment, const Point& point);
    
    //Point and Plane Distance
    float Distance(const Point& point, const PlaneCollision& plane);
    float Distance(const PlaneCollision& plane, const Point& point);
    
    float DistanceSq(const Point& point, const SquareCollision& square);
    float DistanceSq(const SquareCollision& square, const Point& point);

    //Line and Line Distance
    float Distance(const Line& line1, const Line& line2);
    float DistanceSq(const Line& line1, const Line& line2);

    //Line and Segment
    float Distance(const Line& line, const Segment& segment);
    float Distance(const Segment& segment, const Line& line);
    
    float DistanceSq(const Line& line, const Segment& segment);
    float DistanceSq(const Segment& segment, const Line& line);
    
    //Segment and Segment Distance
    float Distance(const Segment& segment1, const Segment& segment2);
    float DistanceSq(const Segment& lhs, const Segment& rhs);
    //Segment and Plane Distance
    float Distance(const Segment& segment, const PlaneCollision& plane);
    float Distance(const PlaneCollision& plane, const Segment& segment);

    /**
     * @tips    return Orthogonal projection
     */
    Vector3 CastToLine(const Line& line, const Point& point);
    Vector3 CastToLine(const Point& point, const Line& line);

    Vector3 CastToPlane(const PlaneCollision& plane, const Point& point);
    Vector3 CastToPlane(const Point& point, const PlaneCollision& plane);

    
    //Point and Point Collision
    bool CollisionReturnFlag(const Point& a, const Point& b);
    
    //Point and LineCollision Collision
    bool CollisionReturnFlag(const Point& p, const Line& l);
    bool CollisionReturnFlag(const Line& l, const Point& p);

    //Point and Segment Collision
    bool CollisionReturnFlag(const Point& p, const Segment& s);
    bool CollisionReturnFlag(const Segment& s, const Point& p);
    
    //LineCollision and LineCollision Collision
    bool CollisionReturnFlag(const Line& l1, const Line& l2);
    
    CollisionData Collision(const Line& line1, const Line& line2);
    
    //Plane and LineCollision Collision
    bool CollisionReturnFlag(const PlaneCollision& p, const Line& l);
    bool CollisionReturnFlag(const Line& l, const PlaneCollision& p);
    
    CollisionData Collision(const PlaneCollision& p, const Line& l);
    CollisionData Collision(const Line& l, const PlaneCollision& p);
    
    //Plane and Segment Collision
    bool CollisionReturnFlag(const PlaneCollision& p, const Segment& s);
    bool CollisionReturnFlag(const Segment& s, const PlaneCollision& p);
    
    CollisionData Collision(const PlaneCollision& p, const Segment& s);
    CollisionData Collision(const Segment& s, const PlaneCollision& p);
    
    //Polygon and Point Collision
    bool CollisionReturnFlag(const PolygonCollision& polygon, const Point& point);
    bool CollisionReturnFlag(const Point& point, const PolygonCollision& polygon);
    
    //Polygon and LineCollision Collision
    bool CollisionReturnFlag(const PolygonCollision& p, const Line& l);
    bool CollisionReturnFlag(const Line& l, const PolygonCollision& p);
    
    CollisionData Collision(const PolygonCollision& p, const Line& l);
    CollisionData Collision(const Line& l, const PolygonCollision& p);
    
    //Polygon and Segment Collision
    bool CollisionReturnFlag(const PolygonCollision& p, const Segment& s);
    bool CollisionReturnFlag(const Segment& s, const PolygonCollision& p);
    
    CollisionData Collision(const PolygonCollision& p, const Segment& s);
    CollisionData Collision(const Segment& s, const PolygonCollision& p);

    //SquareCollision and Point
    bool CollisionReturnFlag(const SquareCollision& s, const Point& p);
    bool CollisionReturnFlag(const Point& p, const SquareCollision& s);
    
    //SquareCollision and Line
    bool CollisionReturnFlag(const SquareCollision& s, const Line& l);
    bool CollisionReturnFlag(const Line& l, const SquareCollision& s);
    
    CollisionData Collision(const SquareCollision& s, const Line& l);
    CollisionData Collision(const Line& l, const SquareCollision& s);
    
    //SquareCollision and Segment
    bool CollisionReturnFlag(const SquareCollision& square, const Segment& segment);
    bool CollisionReturnFlag(const Segment& segment, const SquareCollision& square);
    
    CollisionData Collision(const SquareCollision& square, const Segment& segment);
    CollisionData Collision(const Segment& segment, const SquareCollision& square);
    
    bool CollisionReturnFlag(const SquareCollision& square, const SphereCollision& sphere);
    bool CollisionReturnFlag(const SphereCollision& sphere, const SquareCollision& square);
    
    bool SupSquareSphereColl(const SquareCollision& square, const SphereCollision& sphere, Vector3& nearestPos);
    
    bool CollisionReturnFlag(const SquareCollision& square, const CylinderCollision& cylinder);
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const SquareCollision& square);

    bool CollisionReturnFlag(const SquareCollision& square, const CapsuleCollision& capsule);
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const SquareCollision& square);
    
    //SphereCollision and SphereCollision
    bool CollisionReturnFlag(const SphereCollision& sphere1, const SphereCollision& sphere2);
    
    //Sphere and Point
    bool CollisionReturnFlag(const SphereCollision& sphere, const Point& point);
    bool CollisionReturnFlag(const Point& point, const SphereCollision& sphere);
    
    //SphereCollision and LineCollision
    bool CollisionReturnFlag(const SphereCollision& s, const Line& l);
    bool CollisionReturnFlag(const Line& l, const SphereCollision& s);
    
    CollisionData Collision(const SphereCollision& s, const Line& l);
    CollisionData Collision(const Line& l, const SphereCollision& s);
    
    //SphereCollision and SegmentCollision
    bool CollisionReturnFlag(const SphereCollision& sphere, const Segment& segment);
    bool CollisionReturnFlag(const Segment& segment, const SphereCollision& sphere);
    
    CollisionData Collision(const SphereCollision& sphere, const Segment& l);
    CollisionData Collision(const Segment& l, const SphereCollision& sphere);
    
    //Sphere and Plane
    bool CollisionReturnFlag(const SphereCollision& sphere, const PlaneCollision& plane);
    bool CollisionReturnFlag(const PlaneCollision& plane, const SphereCollision& sphere);

    
    //Sphere and Polygon
    bool CollisionReturnFlag(const SphereCollision& sphere, const PolygonCollision& polygon);
    bool CollisionReturnFlag(const PolygonCollision& polygon, const SphereCollision& sphere);
    
    //Cylinder and Cylinder
    bool CollisionReturnFlag(const CylinderCollision& cylinder1, const CylinderCollision& cylinder2);
    
    //Cylinder and Line
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const Line& line);
    bool CollisionReturnFlag(const Line& line, const CylinderCollision& cylinder);

    //Cylinder and Segment
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const Segment segment);
    bool CollisionReturnFlag(const Segment segment, const CylinderCollision& cylinder);

    
    //Cylinder and Sphere
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const SphereCollision& sphere);
    bool CollisionReturnFlag(const SphereCollision& sphere, const CylinderCollision& cylinder);
    
    //CapsuleCollision and CapsuleCollision
    bool CollisionReturnFlag(const CapsuleCollision& cap1, const CapsuleCollision& cap2);
    
    //Capsule and Point
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const Point& point);
    bool CollisionReturnFlag(const Point& point, const CapsuleCollision& capsule);
    
    //CapsuleCollision and LineCollision
    bool CollisionReturnFlag(const CapsuleCollision& c, const Line& l);
    bool CollisionReturnFlag(const Line& l, const CapsuleCollision& c);
    
    CollisionData Collision(const CapsuleCollision& c, const Line& l);
    CollisionData Collision(const Line& l, const CapsuleCollision& c);
    
    //CapsuleCollision and LineCollision
    bool CollisionReturnFlag(const CapsuleCollision& c, const Segment& s);
    bool CollisionReturnFlag(const Segment& s, const CapsuleCollision& c);
    
    CollisionData Collision(const CapsuleCollision& c, const Segment& s);
    CollisionData Collision(const Segment& s, const CapsuleCollision& c);
    
    //Capsule and Plane
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const PlaneCollision& plane);
    bool CollisionReturnFlag(const PlaneCollision& plane, const CapsuleCollision& capsule);
    
    //SphereCollision and CapsuleCollision
    bool CollisionReturnFlag(const SphereCollision& s, const CapsuleCollision& c);
    bool CollisionReturnFlag(const CapsuleCollision& c, const SphereCollision& s);

    //CubeAABBCollision and Point
    bool CollisionReturnFlag(const AABBCollision& c, const Point& p);
    bool CollisionReturnFlag(const Point& p, const AABBCollision& c);
    
    //CubeAABBCollision and Line
    bool CollisionReturnFlag(const AABBCollision& c, const Line& l);
    bool CollisionReturnFlag(const Line& l, const AABBCollision& c);
    
    CollisionData Collision(const AABBCollision& c, const Line& l);
    CollisionData Collision(const Line& l, const AABBCollision& c);
    
    //CubeAABBCollision and Segment
    bool CollisionReturnFlag(const AABBCollision& c, const Segment& s);
    bool CollisionReturnFlag(const Segment& s, const AABBCollision& c);

    CollisionData Collision(const AABBCollision& c, const Segment& s);
    CollisionData Collision(const Segment& s, const AABBCollision& c);
    
    //CubeAABBCollision and SphereCollision
    bool CollisionReturnFlag(const AABBCollision& c, const SphereCollision& s);
    bool CollisionReturnFlag(const SphereCollision& s, const AABBCollision& c);
    
    //CubeAABBCollision and CapsuleCollision
    bool CollisionReturnFlag(const AABBCollision& cube, const CapsuleCollision& capsul);
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const AABBCollision& cube);

    //CubeAABBCollision and CubeAABBCollision
    bool CollisionReturnFlag(const AABBCollision& cub1, const AABBCollision& cube2);
    
}// namespace myTools

#endif /* Collision_h */

