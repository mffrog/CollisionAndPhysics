//
//  Collision.cpp
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/08.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#include "Collision.h"
#include <math.h>

namespace myTools {    
    
    PlaneCollision CastToPlaneCollision(const Vector3& v1, const Vector3& v2, const Vector3& v3){
        return PlaneCollision(v1,Normalize(cross(v2 - v1, v3 - v2)));
    }
    PlaneCollision CastToPlaneCollision(const PolygonCollision& polygon){
        return PlaneCollision(polygon.p[0], Normalize(cross(polygon.p[1] - polygon.p[0], polygon.p[2] - polygon.p[1])));
    }
    PlaneCollision CastToPlaneCollision(const SquareCollision& square){
        return PlaneCollision(square.p[0], Normalize(cross(square.p[1] - square.p[0], square.p[2] - square.p[1])));
    }
    
    bool IsFront(const PlaneCollision& plane, const Point& point){
        return dot(plane.normal, point - plane.p) >= 0;
    }
    bool IsSharp(const Vector3& v1, const Vector3& v2, const Vector3& v3){
        return dot(v1 - v2, v3 - v2) > 0;
    }
    bool IsInside(const Segment& segment, const Vector3& p){
        return dot(p - segment.p,segment.v) * dot(p - (segment.p + segment.v), segment.v) < 0;
        //return IsSharp(p, segment.p, segment.GetEndPoint()) && IsSharp(p, segment.GetEndPoint(), segment.p);
    }
    
    bool SupPointLineColl(const Point& point, const Line& line, float& t){
        t = 0.0f;
        float len = line.v.LengthSq();
        if(len > 0){
            t = dot(line.v, point - line.p) / len;
        }
        return point == (line.p + t * line.v);
    }
    bool SupLineLineColl(const Line& line1, const Line& line2, float& t1, float& t2, Vector3& pos){
        if(IsParallel(line1.v, line2.v)){
            if(IsParallel(line1.v, line1.p - line2.p)){
                t1 = 0.0f;
                pos = line1.p;
                t2 = dot(pos - line2.p,line2.v) / line2.v.LengthSq();
                return true;
            }
            else {
                t1 = 0.0f;
                t2 = 0.0f;
                return false;
            }
        }
        float Dv1v2 = dot(line1.v, line2.v);
        float Dv1v1 = line1.v.LengthSq();
        float Dv2v2 = line2.v.LengthSq();
        Vector3 lineToLine = line1.p - line2.p;
        t1 = (Dv1v2 * dot(line2.v,lineToLine) - Dv2v2 * dot(line1.v, lineToLine)) / (Dv1v1 * Dv2v2 - Dv1v2 * Dv1v2);
        Vector3 pos1 = line1.p + line1.v * t1 ;
        t2 = dot(line2.v, pos1 - line2.p) / Dv2v2;
        Vector3 pos2 = line2.p + line2.v * t2;
        if(pos1 == pos2){
            pos = pos1;
            return true;
        }
        else {
            return false;
        }
    }
    
    bool SupPlaneLineColl(const PlaneCollision& plane, const Line& line, float& t, Vector3& pos){
        if(!CollisionReturnFlag(plane, line)){
            t = 0.0f;
            return false;
        }
        t = dot((plane.p  - line.p),plane.normal) / dot(line.v, plane.normal);
        pos = line.p + t * line.v;
        return true;
    }
    
    bool SupSphereLineColl(const SphereCollision& sphere, const Line& line,
                           float& t1, float& t2, Vector3& pos1, Vector3& pos2){
        Vector3 toLine = line.p - sphere.position;
        float a = dot(line.v, line.v);
        float b = dot(line.v, toLine);
        float c = dot(toLine, toLine) - sphere.radius * sphere.radius;
        float sol = b * b - a * c;
        if(sol < 0){
            t1 = t1 = 0.0;
            return false;
        }
        sol = sqrtf(sol);
        t1 = (-b - sol) / a;
        t2 = (-b + sol) / a;
        pos1 = line.p + t1 * line.v;
        pos2 = line.p + t2 * line.v;
        return true;
    }
    
    float SupPointLineDist(const Point& point, const Line& line, Point& pos, float& t){
//        t = 0.0f;
//        float lenSq = s.v.LengthSq();
//        if(lenSq > 0){
//            t = (dot(s.v, point - s.p)) / lenSq;
//        }
//        pos = s.p + t * s.v;
//        return (point - pos).Length();
        return sqrtf(SupPointLineDistSq(point, line, pos, t));
    }
    
    float SupPointLineDistSq(const Point& point, const Line& line, Point& pos, float& t){
        t = 0.0f;
        float lenSq = line.v.LengthSq();
        if(lenSq > 0){
            t = (dot(line.v, point - line.p)) / lenSq;
        }
        pos = line.p + t * line.v;
        return (point - pos).LengthSq();
    }
    
    float SupPointSegmentDist(const Point& point, const Segment& segment, float& t, Vector3& pos){
        return sqrtf(SupPointSegmentDistSq(point, segment, t, pos));
//        t = dot(segment.v, point - segment.p) / segment.v.LengthSq();
//        if(t < 0.0f){
//            t = 0.0f;
//            pos = segment.p;
//        }
//        else if( 0.0f <= t && t <= 1.0f){
//            pos = segment.p + t * segment.v;
//        }
//        else {
//            t = 1.0f;
//            pos = segment.GetEndPoint();
//        }
//        return (point - pos).Length();
    }
    float SupPointSegmentDistSq(const Point& point, const Segment& segment, float& t, Vector3& pos){
        t = dot(point - segment.p, segment.v) / segment.v.LengthSq();
        if(t < 0.0f){
            t = 0.0f;
            pos = segment.p;
        }
        else if (1.0f < t){
            t = 1.0f;
            pos = segment.p + segment.v;
        }
        else {
            pos = segment.p + t * segment.v;
        }
        return (point - pos).LengthSq();
    }
    
    
    float SupLineLineDist(const Line& line1, const Line& line2, float& t1, float& t2, Point& p1, Point& p2){
//        if(IsParallel(line1.v, line2.v)){
//            t1 = 0.0f;
//            p1 = line1.p;
//            t2 = dot(line2.v, line1.p - line2.p) / line2.v.LengthSq();
//            p2 = line2.p + line2.v * t2;
//            return (p2 - p1).Length();
//        }
//        float dotV1V2 = dot(line1.v, line2.v);
//        float dotV1V1 = line1.v.LengthSq();
//        float dotV2V2 = line2.v.LengthSq();
//        Vector3 from2To1 = line1.p - line2.p;
//        t1 = ( (dotV1V2 * dot(line2.v, from2To1)) - dotV2V2 * dot(line1.v, from2To1) ) / ( dotV1V1 * dotV2V2 - dotV1V2 * dotV1V2 );
//        p1 = line1.p + (line1.v) * t1;
//
//        t2 = (dot(line2.v, (p1 - line2.p)) / dotV2V2);
//        p2 = line2.p + (line2.v) * t2;
//        return (p2 - p1).Length();
        return sqrtf(SupLineLineDistSq(line1,line2,t1,t2,p1,p2));
    }
    
    float SupLineLineDistSq(const Line& line1, const Line& line2, float& t1, float& t2, Point& p1, Point& p2){
        if(IsParallel(line1.v, line2.v)){
            t1 = 0.0f;
            p1 = line1.p;
            t2 = dot(line2.v, line1.p - line2.p) / line2.v.LengthSq();
            p2 = line2.p + line2.v * t2;
            return (p2 - p1).LengthSq();
        }
        float dotV1V2 = dot(line1.v, line2.v);
        float dotV1V1 = line1.v.LengthSq();
        float dotV2V2 = line2.v.LengthSq();
        Vector3 from2To1 = line1.p - line2.p;
        t1 = ( (dotV1V2 * dot(line2.v, from2To1)) - dotV2V2 * dot(line1.v, from2To1) ) / ( dotV1V1 * dotV2V2 - dotV1V2 * dotV1V2 );
        p1 = line1.p + (line1.v) * t1;
        
        t2 = (dot(line2.v, (p1 - line2.p)) / dotV2V2);
        p2 = line2.p + (line2.v) * t2;
        return (p2 - p1).LengthSq();
    }
    
    float SupPlaneSegmentDist(const PlaneCollision& plane, const Segment& segment, float& t, Point& pos){
        if(SupPlaneLineColl(plane, segment, t, pos)){
            if(0.0f <= t && t <= 1.0f){
                return 0.0f;
            }
            else {
                Clamp(t, 0.0f, 1.0f);
                pos = segment.p + segment.v * t;
                return Distance(pos, plane);
            }
        }
        t = 0.0f;
        pos = segment.p;
        return Distance(segment.p, plane);
    }
    
    //Point and Point
    float Distance(const Point& p1, const Point& p2){
        return (p1 - p2).Length();
    }
    
    //Point and Line Distance
    float Distance(const Point& point, const Line& line){
        Vector3 unit = Normalize(line.v);
        float d = dot(unit, point - line.p);
        return (point - (line.p + d * unit)).Length();
    }
    
    float Distance(const Line& l, const Point& p){
        return Distance(p, l);
    }
    
    float DistanceSq(const Point& point, const Line& line){
        float t = dot(point - line.p,line.v) / line.v.LengthSq();
        return (point - (line.p + line.v * t)).LengthSq();
//
//        Vector3 unit = Normalize(line.v);
//        float d = dot(unit, point - line.p);
//        return (point - (line.p + d * unit)).LengthSq();
    }
    
    float DistanceSq(const Line& l, const Point& p){
        return DistanceSq(p, l);
    }
        
    //Point and Segment Distance
    float Distance(const Point& point, const Segment& segment){
        //TODO: 要検証
        float t = dot(segment.v, point - segment.p) / segment.v.LengthSq();
        if(t < 0.0f){
            return (point - segment.p).Length();
        }
        else if( 0.0f <= t && t <= 1.0f){
            return (point - (segment.p + t * segment.v)).Length();
        }
        else {
            return (point - segment.GetEndPoint()).Length();
        }
    }
    float Distance(const Segment& s, const Point& p){
        return Distance(p, s);
    }
    
    float DistanceSq(const Point& point, const Segment& segment){
        //TODO: 要検証
        float t = dot(segment.v, point - segment.p) / segment.v.LengthSq();
        if(t < 0.0f){
            return (point - segment.p).LengthSq();
        }
        else if( 0.0f <= t && t <= 1.0f){
            return (point - (segment.p + t * segment.v)).LengthSq();
        }
        else {
            return (point - segment.GetEndPoint()).LengthSq();
        }
    }
    
    float DistanceSq(const Segment& segment, const Point& point){
        return DistanceSq(point, segment);
    }
    
    //Point and Plane Distance
    float Distance(const Point& point, const PlaneCollision& plane){
        float dist = dot(point - plane.p, plane.normal);
        return dist >= 0 ? dist : -dist ;
    }
    float Distance(const PlaneCollision& plane, const Point& point){
        return Distance(point, plane);
    }
    
    float DistanceSq(const Point& point, const SquareCollision& square){
        PlaneCollision squPlane(square.p[0],square.GetNormal());
        if(CollisionReturnFlag(square, CastToPlane(squPlane, point))){
            float tmp = Distance(point, squPlane);
            return tmp * tmp;
        }
        const std::vector<Segment>& sides = square.GetSides();
        float dist = DistanceSq(point, sides[0]);
        float tmp = DistanceSq(point, sides[1]);
        if(tmp < dist){
            dist = tmp;
        }
        tmp = DistanceSq(point, sides[2]);
        if(tmp < dist){
            dist = tmp;
        }
        tmp = DistanceSq(point, sides[3]);
        if(tmp < dist){
            return tmp;
        }
        else {
            return dist;
        }
        
    }
    float DistanceSq(const SquareCollision& square, const Point& point){
        return DistanceSq(point, square);
    }
    
    //Line and Line Distance
    float Distance(const Line& line1, const Line& line2){
        Point p[2];
        float t[2];
        return SupLineLineDist(line1, line2, t[0], t[1], p[0], p[1]);
    }
    float DistanceSq(const Line& line1, const Line& line2){
        float t1, t2;
        Vector3 p1,p2;
        if(IsParallel(line1.v, line2.v)){
            t1 = 0.0f;
            p1 = line1.p;
            t2 = dot(line2.v, line1.p - line2.p);
            p2 = line2.p + line2.v * t2;
            return (p2 - p1).LengthSq();
        }
        float dotV1V2 = dot(line1.v, line2.v);
        float dotV1V1 = line1.v.LengthSq();
        float dotV2V2 = line2.v.LengthSq();
        Vector3 from2To1 = line1.p - line2.p;
        t1 = ( (dotV1V2 * dot(line2.v, from2To1)) - dotV2V2 * dot(line1.v, from2To1) ) / ( dotV1V1 * dotV2V2 - dotV1V2 * dotV1V2 );
        p1 = line1.p + (line1.v) * t1;
        
        t2 = (dot(line2.v, (p1 - line2.p)) / dotV2V2);
        p2 = line2.p + (line2.v) * t2;
        return (p2 - p1).LengthSq();
    }
    
    //Line and Segment
    float Distance(const Line& line, const Segment& segment){
        return sqrtf(DistanceSq(line, segment));
    }
    float Distance(const Segment& segment, const Line& line){
        return sqrtf(DistanceSq(line, segment));
    }
    
    float DistanceSq(const Line& line, const Segment& segment){
        if(IsParallel(line.v, segment.v)){
            return (CastToLine(line, segment.p) - segment.p).LengthSq();
        }
        float t[2];
        Vector3 pos[2];
        float distSq = SupLineLineDistSq(line, segment, t[0], t[1], pos[0], pos[1]);
        if(0.0f <= t[1] && t[1] <= 1.0f){
            return distSq;
        }
        Clamp(t[1], 0.0f, 1.0f);
        return DistanceSq(segment.p + segment.v * t[1], line);
    }
    float DistanceSq(const Segment& segment, const Line& line){
        return DistanceSq(line, segment);
    }
    
    //Segment and Segment Distance
    float Distance(const Segment& segment1, const Segment& segment2){
        //TODO: 縮退を考慮するならここに追加
        
        Point pos[2];
        float t[2];
        float len = SupLineLineDist(segment1, segment2, t[0], t[1], pos[0], pos[1]);
        if(0.0f <= t[0] && t[0] <= 1.0f &&
           0.0f <= t[1] && t[1] <= 1.0f){
            return len;
        }
        
        Clamp(t[0], 0.0f, 1.0f);
        pos[0] = segment1.p + segment1.v * t[0];
        len = SupPointLineDist(pos[0], segment2, pos[1], t[1]);
        if(0.0f <= t[1] && t[1] <= 1.0f){
            return len;
        }
        
        Clamp(t[1], 0.0f, 1.0f);
        pos[1] = segment2.p + t[1] * segment2.v;
        len = SupPointLineDist(pos[1], segment1, pos[0], t[0]);
        if(0.0f <= t[0] && t[0] <= 1.0f){
            return len;
        }
        
        Clamp(t[0], 0.0f, 1.0f);
        pos[0] = segment1.p + t[0] * segment1.v;
        return (pos[0] - pos[1]).Length();
    }
    
    
    float DistanceSq(const Segment& lhs, const Segment& rhs){
        Point pos[2];
        float t[2];
        float len = SupLineLineDistSq(lhs, rhs, t[0], t[1], pos[0], pos[1]);
        if(0.0f <= t[0] && t[0] <= 1.0f &&
           0.0f <= t[1] && t[1] <= 1.0f){
            return len;
        }
        
        Clamp(t[0], 0.0f, 1.0f);
        pos[0] = lhs.p + lhs.v * t[0];
        len = SupPointLineDistSq(pos[0], rhs, pos[1], t[1]);
        if(0.0f <= t[1] && t[1] <= 1.0f){
            return len;
        }
        
        Clamp(t[1], 0.0f, 1.0f);
        pos[1] = rhs.p + t[1] * rhs.v;
        len = SupPointLineDistSq(pos[1], lhs, pos[0], t[0]);
        if(0.0f <= t[0] && t[0] <= 1.0f){
            return len;
        }
        
        Clamp(t[0], 0.0f, 1.0f);
        pos[0] = lhs.p + t[0] * lhs.v;
        return (pos[0] - pos[1]).LengthSq();
    }
    
    float SupSegmentSegmentDist(const Segment& lhs, const Segment& rhs, float& t1, float& t2, Vector3& pos1, Vector3& pos2){
        //TODO: 縮退を考慮するならここに追加
        
        float len = SupLineLineDist(lhs, rhs, t1, t2, pos1, pos2);
        if(0.0f <= t1 && t1 <= 1.0f &&
           0.0f <= t2 && t2 <= 1.0f){
            return len;
        }
        
        Clamp(t1, 0.0f, 1.0f);
        pos1 = lhs.p + lhs.v * t1;
        len = SupPointLineDist(pos1, rhs, pos2, t2);
        if(0.0f <= t2 && t2 <= 1.0f){
            return len;
        }
        
        Clamp(t2, 0.0f, 1.0f);
        pos2 = rhs.p + t2 * rhs.v;
        len = SupPointLineDist(pos2, lhs, pos1, t1);
        if(0.0f <= t1 && t1 <= 1.0f){
            return len;
        }
        
        Clamp(t1, 0.0f, 1.0f);
        pos1 = lhs.p + t1 * lhs.v;
        return (pos1 - pos2).Length();
    }
    
    //-----------------------------------------------------------------------------
    //  実装中（start）
    //-----------------------------------------------------------------------------
    
    float SOPlaneAndSegmentDist(const PlaneCollision& plane, const Segment& segment, float& t, Vector3& pos){
        Vector3 v = segment.p - plane.p;
        float d = dot(v, plane.normal);
        if( fabsf(d) < MT_EPSILON){
            t = -1.0f;
            pos = segment.p;
            return 0.0f;
        }
        
        d = dot(plane.normal, segment.v);
        if( fabsf(d) < MT_EPSILON ){
            t = -1.0f;
            pos = segment.p;
            return Distance(segment.p, plane);
        }
        
//        if(!CollisionReturnFlag(plane, Line(segment))){
//            t = -1.0f;
//            pos = segment.p;
//            return Distance(segment.p, plane);
//        }
        
        t = (dot(plane.normal, plane.p - segment.p)) / d;
        
        if(t < 0.0f){
            t = 0.0f;
            pos = segment.p + t * segment.v;
            return Distance(segment.p, plane);
        }
        else if(t > 1.0f){
            t = 1.0f;
            pos = segment.p + t * segment.v;
            return Distance(segment.GetEndPoint(), plane);
        }
        else {
            pos = segment.p + t * segment.v;
            return 0.0f;
        }
    }

    //Segment and Plane Distance
    float Distance(const Segment& segment, const PlaneCollision& plane){
        float bn = dot(plane.p - segment.p, plane.normal);
        //線分が平面状にある場合
        if(fabsf(bn) < MT_EPSILON){
            return 0.0f;
        }
        float vn = dot(plane.normal, segment.v);
        //線分が面と平行な場合
        if(fabsf(vn) < MT_EPSILON){
            return Distance(segment.p, plane);
        }
        
        float t = bn / vn;
        if(t < 0.0f){
            return Distance(segment.p, plane);
        }
        else if (1.0f < t){
            return Distance(segment.p + segment.v, plane);
        }
        else {
            return Distance(segment.p + segment.v * t, plane);
        }
    }
    float Distance(const PlaneCollision& plane, const Segment& segment){
        return Distance(segment, plane);
    }
    
    Vector3 CastToLine(const Line& line, const Point& point){
        //TODO : 要検証
        float t = dot(line.v, point - line.p) / line.v.LengthSq();
        return line.p + t * line.v;
        
//        Vector3 unit = Normalize(line.v);
//        float t = dot(unit, point - line.p);
//        return line.p + t * unit;
    }
    Vector3 CastToLine(const Point& point, const Line& line){
        return CastToLine(line, point);
    }
    Vector3 CastToPlane(const PlaneCollision& plane, const Point& point){
        float t = dot(plane.normal,point - plane.p);
        return point - t * plane.normal;
    }
    Vector3 CastToPlane(const Point& point, const PlaneCollision& plane){
        return CastToPlane(plane, point);
    }
    
    //-----------------------------------------------------------------------------
    //  実装中（end）
    //-----------------------------------------------------------------------------
    
    //Point and Point Collision
    bool CollisionReturnFlag(const Point& a, const Point& b){
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
    
    //Point and LineCollision Collision
    bool CollisionReturnFlag(const Point& p, const Line& l){
        Vector3 ToP = p - l.p;
        Vector3 c = cross(ToP, l.v);
        return fabsf(c.x) < MT_EPSILON && fabsf(c.y) < MT_EPSILON  && fabsf(c.z) < MT_EPSILON ;
    }
    bool CollisionReturnFlag(const Line& l, const Point& p){
        return CollisionReturnFlag(p, l);
    }
    
    
    
    //Point and Segment Collision
    bool CollisionReturnFlag(const Point& p, const Segment& s){
//        float t;
//        Vector3 pos;
//        bool result = SupPointLineColl(p, Line(s.p, s.v), t, pos);
//        if(!result){
//            return false;
//        }
//        if(t < 0.0f || 1.0f < t){
//            return false;
//        }
//        return true;
        
        //TODO : どっちがいいか検証
        
        if(!CollisionReturnFlag(p, Line(s.p,s.v))){
            return false;
        }
        Vector3 v1 = p - s.p;
        Vector3 v2 = p - s.GetEndPoint();
        return v1.LengthSq() <= s.v.LengthSq() && v2.LengthSq() <= s.v.LengthSq();
    }
    
    bool CollisionReturnFlag(const Segment& s, const Point& p){
        return CollisionReturnFlag(p, s);
    }
    
    //LineCollision and LineCollision Collision
    bool CollisionReturnFlag(const Line& l1, const Line& l2){
        Vector3 lineToLine = l2.p - l1.p;
        Vector3 crossproduct = cross(l1.v, lineToLine);
        if(fabsf(crossproduct.x) < MT_EPSILON &&
           fabsf(crossproduct.y) < MT_EPSILON &&
           fabsf(crossproduct.z) < MT_EPSILON ){
            return true;
        }
        
        Vector3 twoVecCross = cross(l1.v, l2.v);
        if(twoVecCross.LengthSq() > 0){
            crossproduct = cross(crossproduct, twoVecCross);
            if(fabsf(crossproduct.x) < MT_EPSILON &&
               fabsf(crossproduct.y) < MT_EPSILON &&
               fabsf(crossproduct.z) < MT_EPSILON ){
                return true;
            }
        }
        return false;
    }
    CollisionData Collision(const Line& line1, const Line& line2){
        float t[2];
        Vector3 pos;
        bool result = SupLineLineColl(line1, line2, t[0], t[1], pos);
        return {pos,result};
        //TODO : どっちが良いか比べる
        
//        if(!CollisionReturnFlag(line1, line2)){
//            return {{},false};
//        }
//        Vector3 unit1 = Normalize(line1.v);
//        Vector3 unit2 = Normalize(line2.v);
//        Vector3 lineToLine = line1.p - line2.p;
//        float linesDot = dot(unit1, unit2);
//        float t = ( - dot(lineToLine, unit1) + linesDot * dot(lineToLine, unit2)) / (1 - linesDot * linesDot);
//        return {Vector3(line1.p + t * unit1),true};
    }
    
    //Plane and Line Collision
    bool CollisionReturnFlag(const PlaneCollision& p, const Line& l){
        Vector3 v = l.p - p.p;
        float d = dot(v, p.normal);
        if( fabsf(d) < MT_EPSILON){
            return true;
        }
        d = dot(p.normal, l.v);
        if( fabsf(d) > 0 ){
            return true;
        }
        return false;
    }
    
    bool CollisionReturnFlag(const Line& l, const PlaneCollision& p){
        return CollisionReturnFlag(p, l);
    }
    
    CollisionData Collision(const PlaneCollision& p, const Line& l){
        if(!CollisionReturnFlag(p, l)){
            return {{},false};
        }
        float t = (dot(p.normal, p.p) - dot(p.normal, l.p)) / dot(p.normal, l.v);
        return {{l.p + t * l.v}, true};
    }
    CollisionData Collision(const Line& l, const PlaneCollision& p){
        return Collision(p, l);
    }
    
    //Plane and Segment Collision
    bool CollisionReturnFlag(const PlaneCollision& p, const Segment& s){
        Vector3 v1 = s.p - p.p;
        Vector3 v2 = s.GetEndPoint() - p.p;
        return (dot(v1, p.normal) * (dot(v2, p.normal))) <= 0;
    }
    bool CollisionReturnFlag(const Segment& s, const PlaneCollision& p){
        return CollisionReturnFlag(p, s);
    }
    
    CollisionData Collision(const PlaneCollision& p, const Segment& s){
        float t;
        Vector3 pos;
        bool result = SupPlaneLineColl(p, s, t, pos);
        if(!result){
            return {{},result};
        }
        if(0.0f <= t && t <= 1.0f){
            return {pos,result};
        }
        else {
            return {{},false};
        }
        
        //TODO: どっちが良いか検証
        
//        CollisionData data = Collision(p,Line({s.p,s.v}));
//        if(!data.hit){
//            return data;
//        }
//        float t;
//        if(SupPointLineColl(data.position, s, t)){
//            if( 0.0f <= t && t <= 1.0f){
//                return data;
//            }
//            else {
//                return {{},false};
//            }
//        }
//        else {
//            return {{},false};
//        }
    }
    CollisionData Collision(const Segment& s, const PlaneCollision& p){
        return Collision(p, s);
    }
    //Polygon and Point Collision
    bool CollisionReturnFlag(const PolygonCollision& polygon, const Point& point){
        Vector3 polygonVec[3] = {
            polygon.p[1] - polygon.p[0],
            polygon.p[2] - polygon.p[1],
            polygon.p[0] - polygon.p[2],
        };
        Vector3 polygonNormal = Normalize(cross(polygonVec[0], polygonVec[1]));
        Vector3 toPoint[3] = {
            point - polygon.p[1],
            point - polygon.p[2],
            point - polygon.p[0],
        };
        return  (polygonNormal - Normalize(cross(polygonVec[0], toPoint[0]))).Length() < MT_EPSILON &&
        (polygonNormal - Normalize(cross(polygonVec[1], toPoint[1]))).Length() < MT_EPSILON &&
        (polygonNormal - Normalize(cross(polygonVec[2], toPoint[2]))).Length() < MT_EPSILON;
    }
    bool CollisionReturnFlag(const Point& point, const PolygonCollision& polygon){
        return CollisionReturnFlag(polygon, point);
    }
    
    
    //Polygon and LineCollision Collision
    bool CollisionReturnFlag(const PolygonCollision& p, const Line& l){
        PlaneCollision plane = CastToPlaneCollision(p);
        bool planeFlag = CollisionReturnFlag(plane, l);
        if (!planeFlag) {
            return false;
        }
        float t = (dot(plane.normal, plane.p) - dot(plane.normal, l.p) )/ dot(plane.normal, l.v);
        Point intersection = l.p + t * l.v ;
        return CollisionReturnFlag(p, intersection);
    }
    
    bool CollisionReturnFlag(const Line& l, const PolygonCollision& p){
        return CollisionReturnFlag(p, l);
    }
    
    CollisionData Collision(const PolygonCollision& p, const Line& l){
        PlaneCollision plane = CastToPlaneCollision(p);
        CollisionData data = Collision(plane,l);
        if(!data.hit){
            return data;
        }
        if(!CollisionReturnFlag(data.position, p)){
            return {{},false};
        }
        return data;
    }
    
    CollisionData Collision(const Line& l, const PolygonCollision& p){
        return Collision(p,l);
    }
    
    //Polygon and Segment Collision
    bool CollisionReturnFlag(const PolygonCollision& p, const Segment& s){
        PlaneCollision plane = CastToPlaneCollision(p);
        float t;
        Vector3 pos;
        bool result = SupPlaneLineColl(plane, s, t, pos);
        if(!result){
            return false;
        }
        if(t < 0.0f || 1.0f < t){
            return false;
        }
        return CollisionReturnFlag(p, pos);
        
        //TODO : ちゃんと動くか確認
        
        
//        if(!CollisionReturnFlag(plane, s)){
//            return false;
//        }
//        float dist[2] = {
//            Distance(s.p, plane),
//            Distance(s.GetEndPoint(), plane),
//        };
//
//        float internalRatio = 1 / (dist[0] + dist[1]);
//        Vector3 onPlanePoint = dist[0] * internalRatio * s.GetEndPoint() + dist[1] * internalRatio * s.p;
//
//        return CollisionReturnFlag(onPlanePoint, p);
    }
    
    bool CollisionReturnFlag(const Segment& s, const PolygonCollision& p){
        return CollisionReturnFlag(p, s);
    }
    
    CollisionData Collision(const PolygonCollision& p, const Segment& s){
        PlaneCollision plane = CastToPlaneCollision(p);
        float t;
        Vector3 pos;
        bool result = SupPlaneLineColl(plane, s, t, pos);
        if(!result){
            return {{},false};
        }
        if(t < 0.0f || 1.0f < t){
            return {{},false};
        }
        if(!CollisionReturnFlag(p, pos)){
            return {{},false};
        }
        return {pos,true};
        
        //TODO : 要チェック
        
//        CollisionData data = Collision(p,Line(s.p, Normalize(s.v)));
//        if(!data.hit){
//            return data;
//        }
//        if(!CollisionReturnFlag(data.position, s)){
//            return {{},false};
//        }
//        else {
//            return data;
//        }
    }
    CollisionData Collision(const Segment& s, const PolygonCollision& p){
        return Collision(p,s);
    }

    //SquareCollision and Point
    bool CollisionReturnFlag(const SquareCollision& s, const Point& p){
        Vector3 squareNormal = s.GetNormal();
        if(Distance(p, PlaneCollision(s.p[0], squareNormal)) >= MT_EPSILON){
            return false;
        }
        Vector3 squareVec[4] = {
            s.p[1] - s.p[0],
            s.p[2] - s.p[1],
            s.p[3] - s.p[2],
            s.p[0] - s.p[3]
        };
        
        Vector3 toPoint[4] = {
            p - s.p[1],
            p - s.p[2],
            p - s.p[3],
            p - s.p[0]
        };
        
        Vector3 normals[4] = {
            cross(squareVec[0], toPoint[0]),
            cross(squareVec[1], toPoint[1]),
            cross(squareVec[2], toPoint[2]),
            cross(squareVec[3], toPoint[3]),
        };
        
        return dot(squareNormal, normals[0]) > 0 &&
        dot(squareNormal, normals[1]) > 0 &&
        dot(squareNormal, normals[2]) > 0 &&
        dot(squareNormal, normals[3]) > 0 ;
        
//        Vector3 result[4] = {
//            Normalize(cross(squareVec[0], toPoint[0])) - squareNormal,
//            Normalize(cross(squareVec[1], toPoint[1])) - squareNormal,
//            Normalize(cross(squareVec[2], toPoint[2])) - squareNormal,
//            Normalize(cross(squareVec[3], toPoint[3])) - squareNormal,
//        };
//
//        for(int i = 0; i < 4; ++i){
//            if(fabsf(result[i].x) > MT_EPSILON ||
//               fabsf(result[i].y) > MT_EPSILON ||
//               fabsf(result[i].z) > MT_EPSILON){
//                return false;
//            }
//        }
        
        return true;
    }
    bool CollisionReturnFlag(const Point& p, const SquareCollision& s){
        return CollisionReturnFlag(s, p);
    }
    
    //SquareCollision and Line
    bool CollisionReturnFlag(const SquareCollision& s, const Line& l){
        PlaneCollision plane(s.p[0],s.GetNormal());

        CollisionData data = Collision(plane, l);
        if(!data.hit){
            return false;
        }
        return CollisionReturnFlag(s, data.position);
    }
    bool CollisionReturnFlag(const Line& l, const SquareCollision& s){
        return CollisionReturnFlag(s, l);
    }
    
    CollisionData Collision(const SquareCollision& s, const Line& l){
        PlaneCollision plane(s.p[0],s.GetNormal());

        CollisionData data = Collision(plane, l);
        if(!data.hit){
            return data;
        }
        if(!CollisionReturnFlag(s, data.position)){
            return {{},false};
        }
        return data;
    }
    CollisionData Collision(const Line& l, const SquareCollision& s){
        return Collision(s, l);
    }

    
    //SquareCollision and Segment
    bool CollisionReturnFlag(const SquareCollision& square, const Segment& segment){
        PlaneCollision plane(square.p[0],square.GetNormal());
        
        float t;
        Vector3 pos;
        bool result = SupPlaneLineColl(plane, segment, t, pos);
        if(!result){
            return false;
        }
        if(t < 0.0f || 1.0f < t){
            return false;
        }
        return CollisionReturnFlag(square, pos);
        
        //TODO : 要チェック
        
//        CollisionData data = Collision(plane, segment);
//        if(!data.hit){
//            return false;
//        }
//
//        return CollisionReturnFlag(square, data.position);
    }
    bool CollisionReturnFlag(const Segment& segment, const SquareCollision& square){
        return CollisionReturnFlag(square, segment);
    }

    
    CollisionData Collision(const SquareCollision& square, const Segment& segment){
        PlaneCollision plane(square.p[0],square.GetNormal()); 
        
        float t;
        Vector3 pos;
        bool result = SupPlaneLineColl(plane, segment, t, pos);
        if(!result){
            return {{},false};
        }
        if(t < 0.0f || 1.0f < t){
            return {{},false};
        }
        if(!CollisionReturnFlag(square, pos)){
            return {{},false};
        }
        return {pos,true};
        
//        CollisionData data = Collision(plane, segment);
//        if(!data.hit){
//            return data;
//        }
//        if(!CollisionReturnFlag(square, data.position)){
//            return {{},false};
//        }
//        return data;
    }
    CollisionData Collision(const Segment& segment, const SquareCollision& square){
        return Collision(square, segment);
    }
    
    //-----------------------------------------------------------------------------
    //  実装中（ここまで）
    //-----------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------
    //  実装中（start）
    //-----------------------------------------------------------------------------
    
    
    bool CollisionReturnFlag(const SquareCollision& square, const SphereCollision& sphere){
        PlaneCollision plane(square.p[0],square.GetNormal());
        if(Distance(sphere.position, plane) > sphere.radius){
            return false;
        }
        
//        float radiusSq = sphere.radius * sphere.radius;
        
//        //四角形が内側にある場合かどうか
//        if((sphere.position - square.p[0]).LengthSq() <= radiusSq ||
//           (sphere.position - square.p[1]).LengthSq() <= radiusSq ||
//           (sphere.position - square.p[2]).LengthSq() <= radiusSq ||
//           (sphere.position - square.p[3]).LengthSq() <= radiusSq
//           ){
//            return true;
//        }
        
        if(CollisionReturnFlag(square, CastToPlane(plane, sphere.position))){
            return true;
        }
        Segment sides[4] = {
            Segment(square.p[0], square.p[1] - square.p[0]),
            Segment(square.p[1], square.p[2] - square.p[1]),
            Segment(square.p[2], square.p[3] - square.p[2]),
            Segment(square.p[3], square.p[0] - square.p[3]),
        };
        return CollisionReturnFlag(sphere, sides[0]) ||
        CollisionReturnFlag(sphere, sides[1]) ||
        CollisionReturnFlag(sphere, sides[2]) ||
        CollisionReturnFlag(sphere, sides[3]);
    }
    bool CollisionReturnFlag(const SphereCollision& sphere, const SquareCollision& square){
        return CollisionReturnFlag(square, sphere);
    }
    
    bool SupSquareSphereColl(const SquareCollision& square, const SphereCollision& sphere, Vector3& nearestPos){
        PlaneCollision plane(square.p[0],square.GetNormal());
        Vector3 spherePos = sphere.position;
        if(Distance(spherePos, plane) > sphere.radius){
            return false;
        }
        
        Vector3 onPlanePos = CastToPlane(plane, spherePos);
        
        if(CollisionReturnFlag(square, onPlanePos)){
            nearestPos = onPlanePos;
            return true;
        }
        Segment sides[4] = {
            Segment(square.p[0], square.p[1] - square.p[0]),
            Segment(square.p[1], square.p[2] - square.p[1]),
            Segment(square.p[2], square.p[3] - square.p[2]),
            Segment(square.p[3], square.p[0] - square.p[3]),
        };

        bool hit = false;
        float radiusSq = sphere.radius * sphere.radius;
        float dist = 0.0f;
        float minDist = 100000.0;
        float t = 0.0f;
        Vector3 pos;
        for(auto& side : sides){
            dist = SupPointSegmentDistSq(spherePos, side, t, pos);
            if(dist <= radiusSq){
                if(!hit){
                    hit = true;
                    minDist = dist;
                    nearestPos = pos;
                }
                else{
                    if(dist < minDist){
                        minDist = dist;
                        nearestPos = pos;
                    }
                }
            }
        }
        return hit;
    }

    bool CollisionReturnFlag(const SquareCollision& square, const CylinderCollision& cylinder){
        float t;
        Vector3 onPlane;
        //Squareないかどうか
        if(SupPlaneLineColl(PlaneCollision(square.p[0], square.GetNormal()), cylinder.line, t, onPlane) &&
           CollisionReturnFlag(square, onPlane)){
            return true;
        }
        
        std::vector<Segment> sides = square.GetSides();
        float radSq = cylinder.radius * cylinder.radius;
        return DistanceSq(cylinder.line, sides[0]) <= radSq ||
        DistanceSq(cylinder.line, sides[1]) <= radSq ||
        DistanceSq(cylinder.line, sides[2]) <= radSq ||
        DistanceSq(cylinder.line, sides[3]) <= radSq;
    }
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const SquareCollision& square){
        return CollisionReturnFlag(square, cylinder);
    }

    
    bool CollisionReturnFlag(const SquareCollision& square, const CapsuleCollision& capsule){
//        //squareがcapsule内にあるかどうかチェック
//        if(CollisionReturnFlag(capsule, square.p[0]) ||
//           CollisionReturnFlag(capsule, square.p[1]) ||
//           CollisionReturnFlag(capsule, square.p[2]) ||
//           CollisionReturnFlag(capsule, square.p[3])
//           ){
//            return true;
//        }
        
        PlaneCollision plane(square.p[0],square.GetNormal());
        float t = 0.0;
        Vector3 pos;
        if(SOPlaneAndSegmentDist(plane, capsule.s, t, pos) > capsule.radius){
            return false;
        }
        //平面とは交差するのであとはSquare内にあるかどうか判定
        //最近点を平面にキャストしてそれがSquare内なら当たってる
        pos = CastToPlane(plane, pos);
        if(CollisionReturnFlag(square, pos)){
            return true;
        }
        
        //最近点がSquare外なら線分と当たり判定する
        Segment sides[4] = {
            Segment(square.p[0], square.p[1] - square.p[0]),
            Segment(square.p[1], square.p[2] - square.p[1]),
            Segment(square.p[2], square.p[3] - square.p[2]),
            Segment(square.p[3], square.p[0] - square.p[3])
        };
        
        return  CollisionReturnFlag(capsule,sides[0]) ||
        CollisionReturnFlag(sides[1], capsule) ||
        CollisionReturnFlag(sides[2], capsule) ||
        CollisionReturnFlag(sides[3], capsule);
    }
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const SquareCollision& square){
        return CollisionReturnFlag(square, capsule);
    }
    
    //-----------------------------------------------------------------------------
    //  実装中（end）
    //-----------------------------------------------------------------------------
    
    //Sphere and Point
    bool CollisionReturnFlag(const SphereCollision& sphere, const Point& point){
        return (sphere.position - point).LengthSq() < sphere.radius * sphere.radius;
    }
    bool CollisionReturnFlag(const Point& point, const SphereCollision& sphere){
        return (sphere.position - point).LengthSq() < sphere.radius * sphere.radius;
    }
    
    //SphereCollision and LineCollision
    bool CollisionReturnFlag(const SphereCollision& s, const Line& l){
        Vector3 spherePos = s.position - l.p;
        float A = dot(l.v, l.v);
        float B = dot(l.v, spherePos);
        float C = dot(spherePos, spherePos) - s.radius * s.radius;
        return (B * B - 4 * A * C) >= 0;
    }
    bool CollisionReturnFlag(const Line& l, const SphereCollision& s){
        return CollisionReturnFlag(s, l);
    }
    
    CollisionData Collision(const SphereCollision& s, const Line& l){
        Vector3 spherePos = s.position - l.p;
        Vector3 unit = Normalize(l.v);
        float A = dot(unit, unit);
        float B = dot(unit, spherePos);
        float C = dot(spherePos, spherePos) - s.radius * s.radius;
        float route = B * B - A * C;
        if(route < 0){
            return {{},false};
        }
        return {{l.p + unit * (( B - sqrtf(route) ) / A)},true};
    }
    CollisionData Collision(const Line& l, const SphereCollision& s){
        return Collision(s,l);
    }
    //SphereCollision and SegmentCollision
    bool CollisionReturnFlag(const SphereCollision& sphere, const Segment& segment){
        return DistanceSq(sphere.position, segment) <= sphere.radius * sphere.radius;
//        float t[2];
//        Vector3 pos[2];
//        bool result = SupSphereLineColl(sphere, segment, t[0], t[1], pos[0], pos[1]);
//        if(!result){
//            return false;
//        }
//
//        if((0.0f <= t[0] && t[0] <= 1.0f) ||
//           (0.0f <= t[1] && t[1] <= 1.0f) ){
//            return true;
//        }
//        return false;
    }
    bool CollisionReturnFlag(const Segment& segment, const SphereCollision& sphere){
        return CollisionReturnFlag(sphere, segment);
    }
    
    CollisionData Collision(const SphereCollision& sphere, const Segment& segment){
        float t[2];
        Vector3 pos[2];
        bool result = SupSphereLineColl(sphere, segment, t[0], t[1], pos[0], pos[1]);
        if(!result){
            return {{},false};
        }
        
        if(0.0f <= t[0] && t[0] <= 1.0f){
            return {pos[0],true};
        }
        if(0.0f <= t[1] && t[1] <= 1.0f){
            return {pos[1],true};
        }
        return {{},false};
        
        //TODO: 要チェック
        
//        CollisionData data = Collision(sphere, Line(segment.p, Normalize(segment.v)));
//        if(!data.hit){
//            return data;
//        }
//        if(!CollisionReturnFlag(segment, data.position)){
//            return {{},false};
//        }
//        else {
//            return data;
//        }
    }
    CollisionData Collision(const Segment& segment, const SphereCollision& sphere){
        return Collision(sphere,segment);
    }
    
    //Sphere and Plane
    bool CollisionReturnFlag(const SphereCollision& sphere, const PlaneCollision& plane){
        return (sphere.position - CastToPlane(plane, sphere.position)).LengthSq() < (sphere.radius * sphere.radius);
    }
    bool CollisionReturnFlag(const PlaneCollision& plane, const SphereCollision& sphere){
        return CollisionReturnFlag(sphere,plane);
    }
    
    //Sphere and Polygon
    bool CollisionReturnFlag(const SphereCollision& sphere, const PolygonCollision& polygon){
        PlaneCollision plane = CastToPlaneCollision(polygon);
        if(Distance(sphere.position, plane) > sphere.radius){
            return false;
        }

        Vector3 onPlane = CastToPlane(plane, sphere.position);
        if(CollisionReturnFlag(polygon, onPlane)){
            return true;
        }
        
        float radSq = sphere.radius * sphere.radius;
        
        if((sphere.radius - polygon.p[0]).LengthSq() < radSq ||
           (sphere.radius - polygon.p[1]).LengthSq() < radSq ||
           (sphere.radius - polygon.p[2]).LengthSq() < radSq ){
            return true;
        }
        
        Segment sides[3] = {
            Segment(polygon.p[0], polygon.p[1] - polygon.p[0]),
            Segment(polygon.p[1], polygon.p[2] - polygon.p[1]),
            Segment(polygon.p[2], polygon.p[0] - polygon.p[2])
        };
        
        return CollisionReturnFlag(sides[0], sphere) ||
        CollisionReturnFlag(sides[1], sphere) ||
        CollisionReturnFlag(sides[2], sphere);
    }
    
    bool CollisionReturnFlag(const PolygonCollision& polygon, const SphereCollision& sphere){
        return CollisionReturnFlag(sphere, polygon);
    }
    
    //SphereCollision and SphereCollision
    bool CollisionReturnFlag(const SphereCollision& sphere1, const SphereCollision& sphere2){
        return (sphere1.position - sphere2.position).LengthSq() <= (sphere1.radius + sphere2.radius) *(sphere1.radius + sphere2.radius)  ;
    }
    
    //Cylinder and Cylinder
    bool CollisionReturnFlag(const CylinderCollision& cylinder1, const CylinderCollision& cylinder2){
        return DistanceSq(cylinder1.line, cylinder2.line) <= (cylinder1.radius + cylinder2.radius) * (cylinder1.radius + cylinder2.radius);
    }
    
    //Cylinder and Line
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const Line& line){
        return DistanceSq(cylinder.line, line) <= cylinder.radius * cylinder.radius;
    }
    bool CollisionReturnFlag(const Line& line, const CylinderCollision& cylinder){
        return CollisionReturnFlag(cylinder, line);
    }
    
    //Cylinder and Segment
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const Segment segment){
        return DistanceSq(cylinder.line, segment) <= cylinder.radius * cylinder.radius;
    }
    bool CollisionReturnFlag(const Segment segment, const CylinderCollision& cylinder){
        return CollisionReturnFlag(cylinder, segment);
    }

    
    bool CollisionReturnFlag(const CylinderCollision& cylinder, const SphereCollision& sphere){
        return DistanceSq(sphere.position, cylinder.line) <= ((cylinder.radius + sphere.radius) * (cylinder.radius + sphere.radius));
    }
    bool CollisionReturnFlag(const SphereCollision& sphere, const CylinderCollision& cylinder){
        return CollisionReturnFlag(cylinder, sphere);
    }

    //CapsuleCollision and CapsuleCollision
    bool CollisionReturnFlag(const CapsuleCollision& cap1, const CapsuleCollision& cap2){
        return Distance(cap1.s, cap2.s) <= cap1.radius + cap2.radius;
    }
    
    
    //Capsule and Point
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const Point& point){
        return DistanceSq(capsule.s, point) < capsule.radius * capsule.radius;
    }
    bool CollisionReturnFlag(const Point& point, const CapsuleCollision& capsule){
        return DistanceSq(capsule.s, point) < capsule.radius * capsule.radius;
    }
    
    
    //CapsuleCollision and LineCollision
    bool CollisionReturnFlag(const CapsuleCollision& c, const Line& l){
        Vector3 pos[2];
        float t[2];
        float len = SupLineLineDistSq(c.s, l, t[0], t[1], pos[0], pos[1]);
        if(t[0] < 0.0f){
            return DistanceSq(c.s.p, l) <= c.radius * c.radius;
        }
        else if(t[0] > 1.0f){
            return DistanceSq(c.s.GetEndPoint(), l) <= c.radius * c.radius;
        }
        else{
            return len <= c.radius * c.radius;
        }
    }
    bool CollisionReturnFlag(const Line& l, const CapsuleCollision& c){
        return CollisionReturnFlag(c, l);
    }
    
    bool CylinderAndLineCollision(CapsuleCollision c, Line l, Vector3& hitS, Vector3& hitE){
        Vector3 offedStart  = c.s.p - l.p;
        
        float Dvv = dot(l.v, l.v);
        float Dsv = dot(c.s.v, l.v);
        float Dpv = dot(offedStart, l.v);
        float Dss = dot(c.s.v, c.s.v);
        float Dps = dot(offedStart, c.s.v);
        float Dpp = dot(offedStart, offedStart);
        float rr = c.radius * c.radius;
        
        if(Dss == 0.0f){
            return false;
        }
        
        float A = Dvv - ((Dsv * Dsv) / Dss);
        float B = Dpv - ((Dps * Dsv) / Dss);
        float C = Dpp - ((Dps * Dps) / Dss) - rr;
        
        //多分円柱と平行で円柱内の線の時
        if(A == 0.0f){
            return false;
        }
        
        float x = B * B - A * C;
        if(x < 0.0f){
            return false;
        }
        
        x = sqrtf(x);
        
        hitS = l.p + ((B - x) / A * l.v);
        hitE = l.p + ((B + x) / A * l.v);
        return true;
    }
    
    CollisionData Collision(const CapsuleCollision& c, const Line& l){
        Vector3 hitS;
        Vector3 hitE;
        if(!CylinderAndLineCollision(c, l, hitS, hitE)){
            //線が円柱と平行で円柱内の場合のチェック
            //近い方の球と判定して返す
            if((c.s.p - l.p).Length() < (c.s.GetEndPoint() - l.p).Length()){
                return Collision(SphereCollision(c.radius,c.s.p), l);
            }
            else {
                return Collision(SphereCollision(c.radius,c.s.GetEndPoint()), l);
            }
            
        }
        //衝突開始点が円柱状の場合そのまま返す
        if(dot( (hitS - c.s.p), ( c.s.GetEndPoint() - c.s.p)) *
           dot( (hitS - c.s.GetEndPoint()), (c.s.p - c.s.GetEndPoint())) >= 0){
            return {hitS, true};
        }
        else {
            //近い方の球と判定して返す
            if((c.s.p - l.p).Length() < (c.s.GetEndPoint() - l.p).Length()){
                return Collision(SphereCollision(c.radius,c.s.p), l);
            }
            else{
                return Collision(SphereCollision(c.radius,c.s.GetEndPoint()), l);
            }
        }
    }
    
    CollisionData Collision(const Line& l, const CapsuleCollision& c){
        return Collision(c, l);
    }
    
    //-----------------------------------------------------------------------------
    //  実装中（ここから）
    //-----------------------------------------------------------------------------
    
    //CapsuleCollision and Segment
    bool CollisionReturnFlag(const CapsuleCollision& c, const Segment& s){
        //間違ってるぞ
//        CollisionData data = Collision(c, Line(s.p,s.v));
//        if(!data.hit){
//            return false;
//        }
//        return CollisionReturnFlag(data.position, s);
        return DistanceSq(c.s, s) <= c.radius * c.radius;
    }
    bool CollisionReturnFlag(const Segment& s, const CapsuleCollision& c){
        return CollisionReturnFlag(c, s);
    }
    
    CollisionData Collision(const CapsuleCollision& c, const Segment& s){
        CollisionData data = Collision(c, Line(s.p,s.v));
        if(!data.hit){
            return data;
        }
        if(!CollisionReturnFlag(data.position, s)){
            return {{},false};
        }
        return data;
    }
    CollisionData Collision(const Segment& s, const CapsuleCollision& c){
        return Collision(c, s);
    }
    
    //Capsule and Plane
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const PlaneCollision& plane){
        return Distance(capsule.s, plane) <= capsule.radius;
    }
    bool CollisionReturnFlag(const PlaneCollision& plane, const CapsuleCollision& capsule){
        return Distance(capsule.s, plane) <= capsule.radius;
    }
    
    //SphereCollision and CapsuleCollision
    bool CollisionReturnFlag(const SphereCollision& s, const CapsuleCollision& c){
        return DistanceSq(s.position, c.s) <= (s.radius + c.radius) * (s.radius + c.radius);
    }
    bool CollisionReturnFlag(const CapsuleCollision& c, const SphereCollision& s){
        return DistanceSq(s.position, c.s) <= (s.radius + c.radius) * (s.radius + c.radius);
    }
    
    
    //CubeAABBCollision and Point
    bool CollisionReturnFlag(const AABBCollision& c, const Point& p){
        if(c.min.x <= p.x && p.x <= c.max.x &&
           c.min.y <= p.y && p.y <= c.max.y  &&
           c.min.z <= p.z && p.z <= c.max.z
           ){
            return true;
        }
        else{
            return false;
        }
    }
    bool CollisionReturnFlag(const Point& p, const AABBCollision& c){
        return CollisionReturnFlag(c, p);
    }
    
    //CubeAABBCollision and Line
    bool CollisionReturnFlag(const AABBCollision& c, const Line& l){

        std::vector<Point> points = c.GetPoints();
        
        SquareCollision square[6];
        square[0].SetPoint(points[0], points[1], points[2], points[3]);
        square[1].SetPoint(points[3], points[2], points[6], points[7]);
        square[2].SetPoint(points[7], points[6], points[5], points[4]);
        square[3].SetPoint(points[4], points[5], points[1], points[0]);
        square[4].SetPoint(points[4], points[0], points[3], points[7]);
        square[5].SetPoint(points[1], points[5], points[6], points[2]);
        if(!CollisionReturnFlag(square[0], l) &&
           !CollisionReturnFlag(square[1], l) &&
           !CollisionReturnFlag(square[2], l) &&
           !CollisionReturnFlag(square[3], l) &&
           !CollisionReturnFlag(square[4], l) &&
           !CollisionReturnFlag(square[5], l)
           ){
            return false;
        }
        return true;
    }
    bool CollisionReturnFlag(const Line& l, const AABBCollision& c){
        return CollisionReturnFlag(c, l);
    }
    
    CollisionData Collision(const AABBCollision& c, const Line& l){
        CollisionData result;
        CollisionData buf;
        
        std::vector<Point> points = c.GetPoints();
        
        SquareCollision square[6];
        square[0].SetPoint(points[0], points[1], points[2], points[3]);
        square[1].SetPoint(points[3], points[2], points[6], points[7]);
        square[2].SetPoint(points[7], points[6], points[5], points[4]);
        square[3].SetPoint(points[4], points[5], points[1], points[0]);
        square[4].SetPoint(points[4], points[0], points[3], points[7]);
        square[5].SetPoint(points[1], points[5], points[6], points[2]);

        for(int i = 0; i < 6; ++i){
            buf = Collision(square[i], l);
            if(buf.hit){
                if(dot(l.v,result.position - l.p) > 0){
                    if(!result.hit){
                        result = buf;
                    }
                    else {
                        if(((l.p - result.position).LengthSq()) > (l.p - buf.position).LengthSq()){
                            result = buf;
                        }
                    }
                }
            }
        }
        return result;
    }
    CollisionData Collision(const Line& l, const AABBCollision& c){
        return Collision(c, l);
    }
    
    //CubeAABBCollision and Segment
    bool CollisionReturnFlag(const AABBCollision& c, const Segment& s){
        
        std::vector<Point> points = c.GetPoints();

        SquareCollision square[6];
        square[0].SetPoint(points[0], points[1], points[2], points[3]);
        square[1].SetPoint(points[3], points[2], points[6], points[7]);
        square[2].SetPoint(points[7], points[6], points[5], points[4]);
        square[3].SetPoint(points[4], points[5], points[1], points[0]);
        square[4].SetPoint(points[4], points[0], points[3], points[7]);
        square[5].SetPoint(points[1], points[5], points[6], points[2]);
        
        return CollisionReturnFlag(square[0], s) ||
        CollisionReturnFlag(square[1], s) ||
        CollisionReturnFlag(square[2], s) ||
        CollisionReturnFlag(square[3], s) ||
        CollisionReturnFlag(square[4], s) ||
        CollisionReturnFlag(square[5], s);
        
//        CollisionData data = Collision(c, Line(s.p,s.v));
//        if(!data.hit){
//            return false;
//        }
//        return CollisionReturnFlag(data.position, s);
    }
    bool CollisionReturnFlag(const Segment& s, const AABBCollision& c){
        return CollisionReturnFlag(c, s);
    }
    
    CollisionData Collision(const AABBCollision& c, const Segment& s){
        if(CollisionReturnFlag(c, s.p) && CollisionReturnFlag(c, s.GetEndPoint())){
            return {s.p,true};
        }
        CollisionData data = Collision(c, Line(s.p, s.v));
        if(!data.hit){
            return data;
        }
        if(!CollisionReturnFlag(data.position, s)){
            return {{},false};
        }
        return data;
    }
    CollisionData Collision(const Segment& s, const AABBCollision& c){
        return Collision(c, s);
    }
    
    //CubeAABBCollision and SphereCollision
    bool CollisionReturnFlag(const AABBCollision& cube, const SphereCollision& sphere){
        Vector3 dist;
        float xDist;
        float yDist;
        float zDist;

        if(cube.min.x > sphere.position.x || sphere.position.x > cube.max.x){
            xDist = cube.min.x - sphere.position.x;
            if(xDist >= 0){
                dist.x = xDist;
            }
            else {
                dist.x = sphere.position.x - cube.max.x;
            }
        }
        
        if(cube.min.y > sphere.position.y || sphere.position.y > cube.max.y){
            yDist = cube.min.y - sphere.position.y;
            if(yDist >= 0){
                dist.y = yDist;
            }
            else {
                dist.y = sphere.position.y - cube.max.y;
            }
        }
        
        if(cube.min.z > sphere.position.z || sphere.position.z > cube.max.z){
            zDist = cube.min.z - sphere.position.z;
            if(zDist >= 0){
                dist.z = zDist;
            }
            else {
                dist.z = sphere.position.z - cube.max.z;
            }
        }
        
        return dist.LengthSq() <= sphere.radius * sphere.radius;
    }
    bool CollisionReturnFlag(const SphereCollision& s, const AABBCollision& c){
        return CollisionReturnFlag(c, s);
    }
    
    //CubeAABBCollision and CapsuleCollision
    bool CollisionReturnFlag(const AABBCollision& cube, const CapsuleCollision& capsule){
        Vector3 capPos(capsule.s.p);
        Vector3 capEndPos(capsule.s.GetEndPoint());
        if(CollisionReturnFlag(cube, capsule.s.p) || CollisionReturnFlag(cube, capsule.s.p + capsule.s.v)){
            return true;
        }
        Vector3 x(1.0f,0.0f,0.0f);
        Vector3 y(0.0f,1.0f,0.0f);
        Vector3 z(0.0f,0.0f,1.0f);

        std::vector<Point> points = cube.GetPoints();
        
        float t;
        Vector3 pos;
        PlaneCollision planeTmp(cube.min,x);
        Segment segTmp(points[4],points[5] - points[4]);
        Segment castSeg[3];
        
        Vector3 castEnd;
        auto makeCastSeg = [&](const PlaneCollision& plane, Segment& castSeg){
            castSeg.p = CastToPlane(plane, capPos);
            castEnd = CastToPlane(plane, capEndPos);
            castSeg.v = castEnd - castSeg.p;
        };
        
        makeCastSeg(planeTmp,castSeg[0]);
        planeTmp.normal = y;
        makeCastSeg(planeTmp,castSeg[1]);
        planeTmp.normal = z;
        makeCastSeg(planeTmp,castSeg[2]);

        planeTmp.normal = -x;
        
        float capRadSq = capsule.radius * capsule.radius;
        
        //左面
        if(SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius ){
            if(DistanceSq(castSeg[2], segTmp) <= capsule.radius * capsule.radius){
                segTmp.p = points[5];
                segTmp.v = points[1] - points[5];
                if(DistanceSq(castSeg[1], segTmp) <= capsule.radius * capsule.radius){
                    if(CollisionReturnFlag(SquareCollision(points[4], points[5], points[1], points[0]), capsule)){
                        return true;
                    }
                }
            }
        }
        planeTmp.normal = -y;
        //下面
        if(SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius){
            segTmp.p = points[1];
            segTmp.v = points[5] - points[1];
            if(DistanceSq(castSeg[0], segTmp) <= capRadSq){
                segTmp.p = points[5];
                segTmp.v = points[6] - segTmp.p;
                if(DistanceSq(castSeg[2], segTmp) <= capRadSq){
                    if(CollisionReturnFlag(capsule,SquareCollision(points[1], points[5], points[6], points[2]))){
                        return true;
                    }
                }
            }
        }
        planeTmp.normal = -z;
        //後ろ面
        if( SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius){
            segTmp.p = points[4];
            segTmp.v = points[5] - points[4];
            if(DistanceSq(castSeg[0], segTmp) <= capRadSq){
                segTmp.p = points[6];
                segTmp.v = points[5] - points[6];
                if(DistanceSq(castSeg[1], segTmp) <= capRadSq){
                    if(CollisionReturnFlag(capsule,SquareCollision(points[7], points[6], points[5], points[4]))){
                        return true;
                    }
                }
            }
        }
        planeTmp.p = cube.max;
        planeTmp.normal = x;
        //右面
        if(SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius){
            segTmp.p = points[6];
            segTmp.v = points[7] - points[6];
            if(DistanceSq(castSeg[2], segTmp) <= capRadSq){
                segTmp.p = points[2];
                segTmp.v = points[6] - points[2];
                if(DistanceSq(castSeg[1], segTmp) <= capRadSq){
                    if(CollisionReturnFlag(SquareCollision(points[3], points[2], points[6], points[7]), capsule)){
                        return true;
                    }
                }
            }
        }
        planeTmp.normal = y;
        //上面
        if(SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius){
            segTmp.p = points[4];
            segTmp.v = points[0] - segTmp.p;
            if(DistanceSq(castSeg[0], segTmp) <= capRadSq){
                segTmp.p = points[4];
                segTmp.v = points[7] - segTmp.p;
                if(DistanceSq(castSeg[2], segTmp) <= capRadSq){
                    if(CollisionReturnFlag(capsule,SquareCollision(points[4], points[0], points[3], points[7]))){
                        return true;
                    }
                }
            }
        }
        planeTmp.normal = z;
        //前面
        if(SupPlaneSegmentDist(planeTmp, capsule.s, t, pos) <= capsule.radius){
            segTmp.p = points[0];
            segTmp.v = points[1] - segTmp.p;
            if(DistanceSq(castSeg[0], segTmp) <= capRadSq){
                segTmp.p = points[1];
                segTmp.v = points[2] - segTmp.p;
                if(DistanceSq(castSeg[1], segTmp) <= capRadSq){
                    if(CollisionReturnFlag(capsule,SquareCollision(points[0], points[1], points[2], points[3]))){
                        return true;
                    }
                }
            }
        }
        return false;
    
//        //内側判定
//        if(CollisionReturnFlag(cube, capsule.s.p)){
//            return true;
//        }
//        //TODO: もっといい方法を考える
//        std::vector<Point> points = cube.GetPoints();
//
//        SquareCollision square[6];
//        square[0].SetPoint(points[0], points[1], points[2], points[3]);
//        square[1].SetPoint(points[3], points[2], points[6], points[7]);
//        square[2].SetPoint(points[7], points[6], points[5], points[4]);
//        square[3].SetPoint(points[4], points[5], points[1], points[0]);
//        square[4].SetPoint(points[4], points[0], points[3], points[7]);
//        square[5].SetPoint(points[1], points[5], points[6], points[2]);
//
//        return CollisionReturnFlag(square[0], capsule) ||
//        CollisionReturnFlag(square[1], capsule) ||
//        CollisionReturnFlag(square[2], capsule) ||
//        CollisionReturnFlag(square[3], capsule) ||
//        CollisionReturnFlag(square[4], capsule) ||
//        CollisionReturnFlag(square[5], capsule) ;
    }
    bool CollisionReturnFlag(const CapsuleCollision& capsule, const AABBCollision& cube){
        return CollisionReturnFlag(cube, capsule);
    }
    
    //CubeAABBCollision and CubeAABBCollision
    bool CollisionReturnFlag(const AABBCollision& cube1, const AABBCollision& cube2){
        if(cube1.max.x < cube2.min.x || cube1.min.x > cube2.max.x)    return false;
        if(cube1.max.y < cube2.min.y || cube1.min.y > cube2.max.y)    return false;
        if(cube1.max.z < cube2.min.z || cube1.min.z > cube2.max.z)    return false;
        return true;
    }

    //-----------------------------------------------------------------------------
    //  実装中（ここまで）
    //-----------------------------------------------------------------------------
    
}// namespace myTools

