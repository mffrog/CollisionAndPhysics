//
//  Primitive.cpp
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/26.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#include "Primitive.h"
#include "Collision.h"

namespace myTools {
    
    PolygonCollision::PolygonCollision(const Vector3& v1, const Vector3& v2, const Vector3 v3){
        p[0] = v1;
        p[1] = v2;
        p[2] = v3;
    }

    void SquareCollision::CulcNormal(){
        normal = Normalize(cross(p[1] - p[0], p[2] - p[1]));
        isCulculated = true;
    }
    SquareCollision::SquareCollision(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4, bool culcNormal){
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        if(culcNormal){
            CulcNormal();
        }
        else {
            isCulculated = false;
        }
    }
    void SquareCollision::SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4){
        p[0] = p1;
        p[1] = p2;
        p[2] = p3;
        p[3] = p4;
        CulcNormal();
    }
    std::vector<Point> SquareCollision::GetPoints() const{
        return std::vector<Point>{p[0],p[1],p[2],p[3]};
    }
    Vector3 SquareCollision::GetNormal() const {
        if(!isCulculated){
            return Normalize(cross(p[1] - p[0], p[2] - p[1]));
        }
        return normal;
    }
    std::vector<Segment> SquareCollision::GetSides() const{
        return std::vector<Segment>{
            Segment(p[0], p[1] - p[0]),
            Segment(p[1], p[2] - p[1]),
            Segment(p[2], p[3] - p[2]),
            Segment(p[3], p[0] - p[3])};
    }
    
    std::vector<Point> AABBCollision::GetPoints() const{
        std::vector<Point> ret;
        ret.push_back(Vector3(min.x,max.y, max.z) + posision);
        ret.push_back(Vector3(min.x,min.y, max.z) + posision);
        ret.push_back(Vector3(max.x,min.y, max.z) + posision);
        ret.push_back(max + posision);
        ret.push_back(Vector3(min.x,max.y,min.z) + posision);
        ret.push_back(min + posision);
        ret.push_back(Vector3(max.x,min.y,min.z) + posision);
        ret.push_back(Vector3(max.x,max.y,min.z) + posision);
        return ret;
    }
    
    Vector3 CapsuleCollision::ToHitPos(const Vector3 hitPos, const Vector3 position) {
        Vector3 onLine = CastToLine(Line(position, s.v), hitPos);
        float d1 = dot(onLine - position,s.v);
        if(d1 * dot(onLine - (position + s.v),s.v) <= 0){
            return Normalize(hitPos - onLine);
        }
        if(d1 < 0){
            return Normalize(hitPos - position);
        }
        else {
            return Normalize(hitPos - (position + s.v));
        }
    }
}

