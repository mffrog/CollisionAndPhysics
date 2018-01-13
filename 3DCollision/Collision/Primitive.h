//
//  Primitive.h
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/09.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#ifndef Primitive_h
#define Primitive_h

#include "Vector.h"
#include <vector>
#include <functional>

namespace myTools {
    struct CollisionData{
        Point position;
        bool hit;
    };
    
    class Collision{
    public:
        virtual Vector3 GetToCenter() = 0;
        void SetCallback(std::function<void()> func){
            hitFunction = func;
        }
        virtual Vector3 ToHitPos(const Vector3 hitPos, const Vector3 position) = 0;
    private:
        std::function<void()> hitFunction = nullptr;
    };
    
    //直線
    struct Line {
        Line() = default;
        Line(const Point& p, const Vector3& v)
        : p(p),v(v){}
        Line(const Line&) = default;
        Line& operator=(const Line&) = default;
        
        Point p;
        Vector3 v;
    };
    
    //線分
    struct Segment : public Line {
        Segment() = default;
        Segment(Vector3 p, Vector3 v):Line(p,v){}
        Segment(const Segment&) = default;
        Segment& operator=(const Segment&) = default;
        Point GetEndPoint() const {
            return p + v;
        }
    };
    
    //平面
    struct PlaneCollision {
        PlaneCollision() = default;
        PlaneCollision(const Point& p, const Vector3& n) : p(p), normal(n){}
        Point p;
        Vector3 normal;
    };
    
    //三角
    struct PolygonCollision {
        PolygonCollision() = default;
        PolygonCollision(const Vector3& v1, const Vector3& v2, const Vector3 v3);
        Point p[3];
    };
    
    //長方形
    struct SquareCollision {
        SquareCollision() = default;
        SquareCollision(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4, bool culcNormal = true);
        Point p[4];
        void SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4);
        void CulcNormal();
        Vector3 GetNormal() const;
        std::vector<Point> GetPoints() const;
        std::vector<Segment> GetSides() const;
    private:
        bool isCulculated = false;
        Vector3 normal;
    };
    
    //キューブ
    struct AABBCollision {
        Vector3 max;
        Vector3 min;
        Vector3 posision;
        std::vector<Point> GetPoints() const ;
    };
    
    struct CubeCollision {
        Vector3 direct[3];
        Vector3 scale;
        Vector3 position;
    };
    
    //球
    struct SphereCollision : public Collision{
        SphereCollision() = default;
        SphereCollision(float radius, Vector3 p) : radius(radius), position(p){}
        Vector3 GetToCenter() override {
            return Vector3(0.0f,0.0f,0.0f);
        }
        Vector3 ToHitPos(const Vector3 hitPos, const Vector3 position) override {
            return Normalize(hitPos - position);
        }
        float radius;
        Vector3 position;
    };
    
    struct DomeCollision {
        float minRadius;
        float maxRadius;
        Vector3 position;
    };
    
    //円柱
    struct CylinderCollision {
        CylinderCollision() = default;
        CylinderCollision(const float& radius, const Line& line) : radius(radius), line(line){}
        float radius;
        Line line;
    };
    
    //カプセル
    struct CapsuleCollision : public Collision{
        CapsuleCollision() = default;
        CapsuleCollision(float radius,Vector3 pos, Vector3 length) : radius(radius), s(pos,length){}
        Vector3 GetToCenter() override{
            return s.v * 0.5f;
        }
        Vector3 ToHitPos(const Vector3 hitPos, const Vector3 position) override;
        Segment s;
        float radius;
    };
}// namespace myTools

#endif /* Primitive_h */

