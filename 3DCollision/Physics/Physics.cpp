//
//  Physics.cpp
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/16.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#include "Physics.h"
#include "Quaternion.h"
#include <math.h>
namespace myTools{
    float Physics::maxVelocity = 1000.0f;
    void Physics::Update(float delta, bool isAccelReset){
        prePos = position + velocity * delta + 0.5f * acceleration * delta * delta;
        preVel = velocity + delta * acceleration;
        
        if(isAccelReset){
            acceleration.x = acceleration.y = acceleration.z = 0 ;
        }
    }
    
    
    Vector3 Physics::GetNewPosition(float delta){
        return position + velocity * delta + 0.5f * acceleration * delta * delta;
    }
    Vector3 Physics::GetNewVelocity(float delta){
        return velocity + acceleration * delta;
    }
    void Physics::SetPosition(const Vector3& position, bool isSmooth){
        if(isSmooth){
            velocity = position - this->position;
        }
        else {
            velocity.x = velocity.y = velocity.z = 0;
            acceleration.x = acceleration.y = acceleration.z = 0;
            this->position = position;
        }
    }
    void Physics::SetVelocity(const Vector3& velocity){
        this->velocity = velocity;
    }
    void Physics::SetAcceleration(const Vector3& acceleration){
        this->acceleration = acceleration;
    }
    
    void Physics::AddPosition(const Vector3& difPos, bool isSmooth){
        if(isSmooth){
            velocity = difPos - this->position;
        }
        else {
            velocity.x = velocity.y = velocity.z = 0;
            acceleration.x = acceleration.y = acceleration.z = 0;
            position += difPos;
        }
    }
    
    void Physics::AddVelocity(const Vector3& difVel){
        velocity += difVel;
    }
    void Physics::AddAcceleration(const Vector3& difAcc){
        acceleration += difAcc;
    }
    
    void Physics::AddForce(const Vector3& force){
        acceleration += force * massRate;
    }
    void Physics::AddImpulse(const Vector3& impulse){
        velocity += impulse * massRate;
    }
    
    void Physics::Fix(){
//        Vector3 posFixVec;
//        if(posSum != 0){
//            float posSumRate = 1 / posSum;
//            for(auto& fix : posFixes){
//                posFixVec += fix * (fix.LengthSq()) * posSumRate;
//            }
//        }
//        position = posFixVec + prePos;
        position = prePos;
        
//        Vector3 velFixVec;
//        for(auto& fix : velFixes){
//            preVel += fix;
//        }
        velocity = preVel;
        velocity.x = velocity.x > maxVelocity ? maxVelocity : velocity.x;
        velocity.y = velocity.y > maxVelocity ? maxVelocity : velocity.y;
        velocity.z = velocity.z > maxVelocity ? maxVelocity : velocity.z;

//        //規定値以下なら速度をなくす(荒ぶり防止)
//        static float velThreshold = 1.0f;
//        for (int i = 0 ; i < 3; ++i) {
//            velocity[i] = fabsf(velocity[i]) < velThreshold ? 0.0f : velocity[i];
//        }
        ResetFix();
    }
    
    void Physics::PreFix(){
        Vector3 posFixVec;
        if(posSum != 0){
            float posSumRate = 1 / posSum;
            for(auto& fix : posFixes){
                posFixVec += fix * (fix.LengthSq()) * posSumRate;
            }
        }
        prePos += posFixVec;
        
        Vector3 velFixVec;
        for(auto& fix : velFixes){
            preVel += fix;
        }
        restrictVectores.clear();
//        auto itr = restrictVectores.begin();
//        while (itr != restrictVectores.end()) {
//            if((dot(*itr, preVel)) < 0){
//                itr = restrictVectores.erase(itr);
//            }
//            else {
//                ++itr;
//            }
//        }
        //ResetFix();
    }
    
    Vector3 Physics::CulcRestrictPower(const Vector3& impulse){
        Vector3 restrictPower;
        float d;
        //TODO : もうちょっと考えた方がいいかも
        for(auto& n : restrictVectores){
            d = dot(n,impulse);
            //内積が正のときは制限方向と同じ方から押されることになるので制限しない
            if(d < 0){
                restrictPower -= d * n;
            }
        }
        return restrictPower;
    }
    
    Vector3 CulcVel(const Physics& phys){
        return (phys.GetPrePos() - phys.GetPosition());
    }
    
    //Sphere and Sphere HitTime
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere1,
                          const MoveCollData<SphereCollision>& sphere2){
        
        HitData ret;
        
        //移動前に当たってるかチェック
        if(CollisionReturnFlag(sphere1.collision, sphere2.collision)){
            ret.hit = true;
            ret.time = 0.0f;
            Vector3 from1To2 = sphere2.collision.position - sphere1.collision.position;
            ret.hitPos = from1To2 * (sphere1.collision.radius) / (sphere1.collision.radius + sphere2.collision.radius) + sphere1.collision.position;
            ret.length =  (sphere1.collision.radius + sphere2.collision.radius) - from1To2.Length();
            ret.hitNormal = Normalize(from1To2);
            return ret;
        }
        
        Vector3 lhsPos = sphere1.phys.GetPosition();
        Vector3 rhsPos = sphere2.phys.GetPosition();
        
        Vector3 start = lhsPos - rhsPos;
        Vector3 end = sphere1.phys.GetPrePos() - sphere2.phys.GetPrePos();
        Vector3 vel = end - start;
        
        float a = vel.LengthSq();
        float b = dot(start, vel);
        float c = start.LengthSq() - (sphere1.collision.radius + sphere2.collision.radius) * (sphere1.collision.radius + sphere2.collision.radius);
        float ans = b * b - a * c;
        if(ans < 0 || a  == 0){
            return ret;
        }
        
        
        
        //めり込み量は貫通していた場合のめり込み距離を定義してから追加
        //        //めり込み算出
        //        Vector3 pos = sphere2.phys.GetPrePos();
        //        if(dot(sphere1.phys.GetPosition() - pos, sphere1.phys.GetPrePos() - pos) >= 0){
        //            ret.length = sphere1.collision.radius + sphere2.collision.radius - (sphere1.phys.GetPrePos() - pos).Length();
        //        }
        //        else {
        //            ret.length = (sphere1.phys.GetPrePos() - pos).Length() + sphere1.collision.radius + sphere2.collision.radius;
        //        }
        
        
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(t < 0.0f || 1.0f < t){
                ret.hit = false;
                ret.time = 0.0f;
                ret.length = 0.0f;
                return ret;
            }
            
            ret.hit = true;
            ret.time = t;
            Vector3 pos = sphere2.phys.GetPosition() + CulcVel(sphere2.phys) * t;
            ret.hitPos = pos + (start + vel * t) * (sphere2.collision.radius) / (sphere2.collision.radius + sphere1.collision.radius);
            ret.hitNormal = Normalize(ret.hitPos - pos);
        }
        
        ans = sqrtf(ans);
        float t[2] = {
            (-b - ans)/a,
            (-b + ans)/a
        };
        
        if(0.0f <= t[0] && t[0] <= 1.0f){
            ret.hit = true;
            ret.time = t[0];
            Vector3 pos = sphere2.phys.GetPosition() + CulcVel(sphere2.phys) * t[0];
            ret.hitPos = pos + (start + vel * t[0]) * (sphere2.collision.radius) / (sphere2.collision.radius + sphere1.collision.radius);
            ret.hitNormal = Normalize(ret.hitPos - pos);
            
        }
        return ret;
        
        //        if((t[0]< 0.0f || 1.0f < t[0]) &&
        //           (t[1] < 0.0f || 1.0f < t[1])){
        //            ret.hit = false;
        //            ret.time = 0.0f;
        //            ret.length = 0.0f;
        //        }
        //        ret.hit = true;
        //        if(0.0f <= t[0] && t[0] <= 1.0f){
        //            ret.time = t[0];
        //            return ret;
        //        }
        //        else {
        //            ret.time = 0.0f;
        //            return ret;
        //        }
    }
    
    Vector3 SupParaSegCenter(const Segment& lhs, const Segment& rhs, Vector3& pos1, Vector3& pos2){
        Vector3 endpoints[2][2] = {
            {lhs.p, lhs.p + lhs.v},
            {rhs.p, rhs.p + rhs.v}
        };
        bool inside[2][2] = {
            {IsInside(rhs, endpoints[0][0]), IsInside(rhs, endpoints[0][1])},
            {IsInside(lhs, endpoints[1][0]), IsInside(lhs, endpoints[1][1])}
        };
        
        if(inside[0][0] && inside[0][1]){
            pos1 = (endpoints[0][0] + endpoints[0][1]) * 0.5f;
            pos2 = CastToLine(rhs, pos1);
            return (pos1 + pos2);
        }
        if(inside[1][0] && inside[1][1]){
            pos2 = (endpoints[1][0] + endpoints[1][1]) * 0.5f;
            pos1 = CastToLine(lhs, pos2);
            return (pos1 + pos2) * 0.5f;
        }
        
        if((inside[0][0] || inside[0][1]) && (!inside[0][0] || !inside[0][1])){
            const Vector3& insidePos = inside[0][0] ? endpoints[0][0] : endpoints[0][1];
            const Vector3& outSidePos = inside[0][0] ? endpoints[0][1] : endpoints[0][0];
            const Vector3& sharpPos = IsSharp(endpoints[1][0], insidePos, outSidePos) ? endpoints[1][0] : endpoints[1][1];
            Vector3 result = (insidePos + sharpPos) * 0.5f;
            pos1 = CastToLine(lhs, result);
            pos2 = CastToLine(rhs, result);
            return result;
        }
        else if((inside[1][0] || inside[1][1]) && (!inside[1][0] || !inside[1][1])){
            const Vector3& insidePos = inside[1][0] ? endpoints[1][0] : endpoints[1][1];
            const Vector3& outSidePos = inside[1][0] ? endpoints[1][1] : endpoints[1][0];
            const Vector3& sharpPos = IsSharp(endpoints[0][0], insidePos, outSidePos) ? endpoints[0][0] : endpoints[0][1];
            Vector3 result = (insidePos + sharpPos) * 0.5f;
            pos1 = CastToLine(lhs, result);
            pos2 = CastToLine(rhs, result);
            return result;
        }
        else {
            Vector3 lhsNear = (endpoints[1][0] - endpoints[0][0]).LengthSq() < (endpoints[1][0] - endpoints[0][1]).LengthSq() ? endpoints[0][0] : endpoints[0][1];
            Vector3 rhsNear = (endpoints[0][0] - endpoints[1][0]).LengthSq() < (endpoints[0][0] - endpoints[1][1]).LengthSq() ? endpoints[1][0] : endpoints[1][1];
            Vector3 result = (lhsNear + rhsNear) * 0.5f;
            pos1 = CastToLine(lhs, result);
            pos2 = CastToLine(rhs, result);
            return (pos1 + pos2) * 0.5f;
        }
    }
    
    Vector3 SupParaCylinderHitPoint(const CylinderCollision& lhs, const CylinderCollision& rhs, const Vector3& pos1, const Vector3& pos2){
        Vector3 endpoints[2][2] = {
            {lhs.line.p, lhs.line.p + lhs.line.v},
            {rhs.line.p, rhs.line.p + rhs.line.v},
        };
        
        Segment seg(rhs.line.p,rhs.line.v);
        
        bool isInside[2][2] ;
        isInside[0][0] = IsInside(seg, endpoints[0][0]);
        isInside[0][1] = IsInside(seg, endpoints[0][1]);
        seg.p = lhs.line.p;
        seg.v = lhs.line.v;
        isInside[1][0] = IsInside(seg, endpoints[1][0]);
        isInside[1][1] = IsInside(seg, endpoints[1][1]);
        
        Vector3 inLine[2];
        
        if(isInside[0][0] && isInside[0][1]){
            inLine[0] = endpoints[0][0];
            inLine[1] = endpoints[0][1];
            return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
        }
        else if(isInside[1][0] && isInside[1][1]){
            inLine[0] = endpoints[1][0];
            inLine[1] = endpoints[1][1];
            return (inLine[0] + inLine[1]) * 0.5f + (pos1 - pos2) * (rhs.radius/(lhs.radius + rhs.radius));
        }
        
        if((isInside[0][0] || isInside[0][1]) && (!isInside[0][0] || !isInside[0][1])){
            if(isInside[0][0]){
                inLine[0] = endpoints[0][0];
                inLine[1] = IsSharp(endpoints[1][0], inLine[0], endpoints[0][1]) ? endpoints[1][0] : endpoints[1][1];
                inLine[1] = CastToLine(lhs.line, inLine[1]);
                return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
            }
            else {
                inLine[0] = endpoints[0][1];
                inLine[1] = IsSharp(endpoints[1][0], inLine[0], endpoints[0][0]) ? endpoints[1][0] : endpoints[1][1];
                inLine[1] = CastToLine(lhs.line, inLine[1]);
                return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
            }
        }
        
        if((isInside[1][0] || isInside[1][1]) && (!isInside[1][0] || !isInside[1][1])){
            if(isInside[1][0]){
                inLine[1] = endpoints[1][0];
                inLine[0] = IsSharp(endpoints[0][0], inLine[1], endpoints[1][1]) ? endpoints[0][0] : endpoints[0][1];
                inLine[1] = CastToLine(lhs.line, inLine[1]);
                return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
            }
            else {
                inLine[1] = endpoints[1][1];
                inLine[0] = IsSharp(endpoints[0][0], inLine[1], endpoints[1][0]) ? endpoints[0][0] : endpoints[0][1];
                inLine[1] = CastToLine(lhs.line, inLine[1]);
                return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
            }
        }
        
        inLine[0] = (endpoints[0][0] + endpoints[0][1]) * 0.5f;
        inLine[1] = (endpoints[1][0] + endpoints[1][1]) * 0.5f;
        inLine[1] = CastToLine(lhs.line, inLine[1]);
        return (inLine[0] + inLine[1]) * 0.5f + (pos2 - pos1) * (lhs.radius/(lhs.radius + rhs.radius));
    }
    
    //Cylinder and Cylinder HitTime
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder1,
                          const MoveCollData<CylinderCollision>& cylinder2){
        
        HitData ret;
        
        float t[2];
        Point pos[2];
        float dist = SupLineLineDist(cylinder1.collision.line, cylinder2.collision.line, t[0], t[1], pos[0], pos[1]);
        float radiusSum = cylinder1.collision.radius + cylinder2.collision.radius;
        
        bool isParallel = IsParallel(cylinder1.collision.line.v, cylinder2.collision.line.v);
        
        //移動まえにすでに当たっているかどうかチェック
        if(dist <= radiusSum){
            ret.hit = true;
            ret.time = 0.0f;
            if(isParallel){
                ret.hitPos = SupParaCylinderHitPoint(cylinder1.collision, cylinder2.collision, pos[0], pos[1]);
            }
            else {
                Vector3 from1To2 = pos[1] - pos[0];
                ret.hitPos = pos[0] + ((cylinder1.collision.radius)/radiusSum) * from1To2;
            }
            ret.length = radiusSum - dist;
            ret.hitNormal = Normalize(pos[1] - pos[0]);
            return ret;
        }
        
        
        //これでいける気がする
        Vector3 lhsStart = pos[0];
        Vector3 rhsStart = pos[1];
        Vector3 start = pos[0] - pos[1];
        SupLineLineDist(Line(cylinder1.phys.GetPrePos(),cylinder1.collision.line.v),
                        Line(cylinder2.phys.GetPrePos(),cylinder2.collision.line.v), t[0], t[1], pos[0], pos[1]);
        Vector3 lhsEnd = pos[0];
        Vector3 rhsEnd = pos[1];
        Vector3 end = pos[0] - pos[1];
        Vector3 vel = end - start;
        
        //        Vector3 cyl1Pos = pos[0];
        //        Vector3 cyl1Vel = CulcVel(cylinder1.phys) - CulcVel(cylinder2.phys);
        //        Vector3 onLineStart = pos[1];
        //
        //        Line line1(pos[0] + cyl1Vel,cylinder1.collision.line.v);
        //        SupLineLineDist(line1, cylinder2.collision.line, t[0], t[1], pos[0], pos[1]);
        //
        //        Vector3 onLineEnd = pos[1];
        //
        //        Vector3 start = cyl1Pos - onLineStart;
        //        Vector3 end = pos[0] - onLineEnd;
        //        Vector3 vel = end - start;
        
        float a = vel.LengthSq();
        float b = dot(start,vel);
        float c = start.LengthSq() - (radiusSum * radiusSum);
        
        
        float ans = b * b - a * c;
        if(ans < 0.0f || a == 0.0f){
            return ret;
        }
        
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(t < 0.0f || 1.0f < t){
                return ret;
            }
            ret.hit = true;
            ret.time = t;
            //再利用
            pos[0] = lhsStart + (lhsEnd - lhsStart) * t;
            pos[1] = rhsStart + (rhsEnd - rhsStart) * t;
            ret.hitNormal = Normalize(pos[1] - pos[0]);
            if(isParallel){
                ret.hitPos = SupParaCylinderHitPoint
                (
                 CylinderCollision(cylinder1.collision.radius, Line(pos[0], cylinder1.collision.line.v)),
                 CylinderCollision(cylinder2.collision.radius,Line(pos[1], cylinder2.collision.line.v)),
                 pos[0], pos[1]
                 );
            }
            else {
                ret.hitPos = ((cylinder2.collision.radius)/radiusSum ) * (start + vel * t) + pos[1];
            }
            return ret;
        }
        
        ans = sqrtf(ans);
        
        t[0] = (-b - ans) / a;
        t[1] = (-b + ans) / a;
        
        if(0.0f <= t[0] && t[0] <= 1.0f){
            ret.hit = true;
            ret.time = t[0];
            //再利用
            pos[0] = lhsStart + (lhsEnd - lhsStart) * t[0];
            pos[1] = rhsStart + (rhsEnd - rhsStart) * t[0];
            ret.hitNormal = Normalize(pos[1] - pos[0]);
            if(isParallel){
                
                ret.hitPos = SupParaCylinderHitPoint
                (
                 CylinderCollision(cylinder1.collision.radius, Line(pos[0], cylinder1.collision.line.v)),
                 CylinderCollision(cylinder2.collision.radius,Line(pos[1], cylinder2.collision.line.v)),
                 pos[0], pos[1]
                 );
            }
            else {
                ret.hitPos = ((cylinder2.collision.radius)/radiusSum ) * (start + vel * t[0]) + pos[1];
            }
        }
        
        return ret;
    }
    
    //Sphere and Cylinder
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere,
                          const MoveCollData<CylinderCollision>& cylinder){
        
        HitData ret;
        
        float radiusSum = cylinder.collision.radius + sphere.collision.radius;
        if(CollisionReturnFlag(cylinder.collision, sphere.collision)){
            ret.hit = true;
            ret.time = 0.0f;
            Vector3 onLine = CastToLine(cylinder.collision.line, sphere.collision.position);
            Vector3 w = sphere.collision.position - onLine;
            ret.hitPos = (cylinder.collision.radius / radiusSum) * w + onLine;
            ret.length = radiusSum - w.Length();
            ret.hitNormal = Normalize(w);
            return ret;
        }
        
        Vector3 spherePos = sphere.phys.GetPosition();
        Vector3 spherePrePos = sphere.phys.GetPrePos();
        
        Vector3 onLineStart = CastToLine(cylinder.collision.line, spherePos);
        Vector3 onLineEnd = CastToLine(Line(cylinder.phys.GetPrePos(),cylinder.collision.line.v), spherePrePos);
        
        Vector3 start = spherePos - onLineStart;
        Vector3 end = spherePrePos - onLineEnd;
        Vector3 vel = end - start;
        
        //        Vector3 spherePos = sphere.phys.GetPosition();
        //        Vector3 sphereVel = CulcVel(sphere.phys) - CulcVel(cylinder.phys);
        //        Vector3 onLineStart = CastToLine(cylinder.collision.line, spherePos);
        //        Vector3 onLineEnd = CastToLine(cylinder.collision.line, spherePos + sphereVel);
        //
        //        Vector3 start = spherePos - onLineStart;
        //        Vector3 end = spherePos + sphereVel - onLineEnd;
        //        Vector3 vel = end - start;
        
        float a = vel.LengthSq();
        float b = dot(start,vel);
        float c = start.LengthSq() - radiusSum * radiusSum;
        
        float ans = b * b - a * c;
        
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(0.0f <= t && t <= 1.0f){
                ret.hit = true;
                ret.time = t;
                Vector3 pos = (onLineStart + (onLineEnd - onLineStart) * t);
                ret.hitPos = (cylinder.collision.radius / radiusSum) * (start + vel * t) + pos;
                ret.hitNormal = Normalize(ret.hitPos - pos);
            }
            return ret;
        }
        
        ans = sqrtf(ans);
        float t[2] = {
            (-b - ans) / a,
            (-b + ans) / a
        };
        if(0.0f <= t[0] && t[0] <= 1.0f){
            ret.hit = true;
            ret.time = t[0];
            Vector3 pos = (onLineStart + (onLineEnd - onLineStart) * t[0]);
            ret.hitPos = (cylinder.collision.radius / radiusSum) * (start + vel * t[0]) + pos;
            ret.hitNormal = Normalize(ret.hitPos - pos);
        }
        
        return ret;
    }
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder,
                          const MoveCollData<SphereCollision>& sphere){
        return MoveCollision(sphere, cylinder);
    }
    
    //Cylinder and Capsule
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder,
                          const MoveCollData<CapsuleCollision>& capsule){
        HitData ret;
        HitData data[3];
        data[0] = MoveCollision
        (
         MoveCollData<myTools::SphereCollision>(SphereCollision(capsule.collision.radius,capsule.collision.s.p),capsule.phys),
         cylinder
         );
        Physics endPhys;
        endPhys.SetPosition(capsule.collision.s.GetEndPoint(), false);
        endPhys.SetPrePos(capsule.phys.GetPrePos() + capsule.collision.s.v);
        data[1] = MoveCollision
        (MoveCollData<myTools::SphereCollision>(SphereCollision(capsule.collision.radius,capsule.collision.s.GetEndPoint()),
                                                endPhys),
         cylinder
         );
        
        data[2] = MoveCollision(cylinder, MoveCollData<myTools::CylinderCollision>(CylinderCollision(capsule.collision.radius, capsule.collision.s),capsule.phys));
        if(data[2].hit){
            //カプセルの円柱の範囲内か確認
            Vector3 pos = capsule.phys.GetPosition() + CulcVel(capsule.phys) * data[2].time;
            data[2].hit = IsInside(Segment(pos, capsule.collision.s.v), data[2].hitPos);
        }
        for(auto& d : data){
            if(d.hit){
                if(!ret.hit){
                    ret = d;
                }
                else {
                    if(d.time <= ret.time){
                        ret = d;
                    }
                }
            }
        }
        
        return ret;
        
    }
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& capsule,
                          const MoveCollData<CylinderCollision>& cylinder){
        return MoveCollision(cylinder, capsule);
    }
    
    HitData SupCapSphereAndSphere(const MoveCollData<SphereCollision>& sphere,
                                  const MoveCollData<CapsuleCollision>& capsule){
        HitData ret;
        
        //始点の球
        SphereCollision capSphere(capsule.collision.radius,capsule.phys.GetPosition());
        MoveCollData<SphereCollision> capSphereData;
        capSphereData.collision = capSphere;
        capSphereData.phys = capsule.phys;
        ret = MoveCollision(capSphereData, sphere);
        //終点の球
        capSphere.position = capsule.phys.GetPosition() + capsule.collision.s.v;
        Physics capSpherePhys;
        capSpherePhys.SetPosition(capSphere.position,false);
        capSpherePhys.SetPrePos(capsule.phys.GetPrePos() + capsule.collision.s.v);
        capSphereData.collision = capSphere;
        capSphereData.phys = capSpherePhys;
        HitData buf = MoveCollision(capSphereData, sphere);
        if(ret.hit){
            if(buf.time < ret.time){
                ret = buf;
            }
        }
        return buf;
    }
    
    //Sphere and Capsule HitTime
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere,
                          const MoveCollData<CapsuleCollision>& capsule){
        
        //TODO: いじったよ
        
        HitData ret;
        float radiusSum = capsule.collision.radius + sphere.collision.radius;
        float d;
        Vector3 pos;
        float distSq = SupPointSegmentDistSq(sphere.collision.position, capsule.collision.s, d, pos);
        //移動前に当たっているかチェック
        if(distSq <= radiusSum * radiusSum){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = (sphere.collision.position - pos) * (capsule.collision.radius / radiusSum) + pos;
            ret.hitNormal = Normalize(ret.hitPos - pos);
            ret.length = radiusSum - sqrtf(distSq);
            return ret;
        }
        
        //
        //TODO : どっちが良いか考える
        MoveCollData<CylinderCollision> cylinder;
        cylinder.collision = CylinderCollision(capsule.collision.radius, capsule.collision.s);
        cylinder.phys = capsule.phys;
        
        ret = MoveCollision(cylinder, sphere);
        if(ret.hit){
            Vector3 pos = capsule.phys.GetPosition() + CulcVel(capsule.phys) * ret.time;
            if(!IsInside(Segment(pos, capsule.collision.s.v), ret.hitPos)){
                return SupCapSphereAndSphere(sphere, capsule);
            }
            else {
                return ret;
            }
        }
        return ret;
        //
        
        Vector3 spherePos = sphere.phys.GetPosition();
        
        Vector3 capPos = capsule.phys.GetPosition();
        Vector3 capDire = capsule.collision.s.v;
        
        Vector3 sphereVel = CulcVel(sphere.phys) - CulcVel(capsule.phys);
        Vector3 unit = Normalize(capsule.collision.s.v);
        Vector3 onLineVel = dot(unit, sphereVel) * unit;
        Vector3 onLineStartPos = CastToLine(capsule.collision.s, spherePos);
        
        Vector3 start = spherePos - onLineStartPos;
        Vector3 end = (spherePos + sphereVel) - (onLineStartPos + onLineVel);
        Vector3 vel = end - start;
        
        float a = vel.LengthSq();
        float b = dot(start,vel);
        float c = start.LengthSq() - ((sphere.collision.radius + capsule.collision.radius) * (sphere.collision.radius + capsule.collision.radius));
        
        float ans = b * b - a * c;
        
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(t < 0.0f || 1.0f < t){
                return ret;
            }
            
            Vector3 onLinePos = onLineStartPos + t * onLineVel;
            //線分内にあるかどうか判定
            if(dot(onLinePos - capPos, capDire) * dot(onLinePos - (capPos + capDire), capDire) < 0){
                ret.hit = true;
                ret.time = t;
                return ret;
            }
            
            
            //間違ってそうやからチェックするように
            /*
             上にサポート関数作った
             //線分外の場合端点の球とそれぞれ判定
             //始点の球
             SphereCollision capSphere(capsule.collision.radius,capPos);
             MoveCollData<SphereCollision> capSphereData;
             capSphereData.collision = capSphere;
             capSphereData.phys = capsule.phys;
             ret = MoveCollision(capSphereData, sphere);
             if(ret.hit){
             return ret;
             }
             //終点の球
             capSphere.position = capPos + capDire;
             Physics capSpherePhys;
             capSpherePhys.SetPosition(capSphere.position);
             capSpherePhys.SetPrePos(capsule.phys.GetPrePos() + capDire);
             capSphereData.collision = capSphere;
             capSphereData.phys = capSpherePhys;
             return MoveCollision(capSphereData, sphere);
             */
            
            return SupCapSphereAndSphere(sphere, capsule);
        }
        
        ans = sqrtf(ans);
        float t[2] = {
            (-b - ans) / a,
            (-b + ans) / a
        };
        
        //衝突開始時刻がこのステップ内
        if(0.0f <= t[0] && t[0] < 1.0f){
            Vector3 onLinePos = onLineStartPos + t[0] * onLineVel;
            //線分内にあるかどうか判定
            if(dot(onLinePos - capPos, capDire) * dot(onLinePos - (capPos + capDire), capDire) < 0){
                ret.hit = true;
                ret.time = t[0];
                return ret;
            }
        }
        
        /*
         t2( (-b + ans) / a )は通り抜ける時刻なので
         それが線分内であろうが外であろうが
         カプセルとの衝突開始時刻にはならないので球と判定しておしまい
         あってるか確認
         
         t1が範囲外でt2が範囲内の場合円柱とは t = 0.0f の時点で当たっているのでそれの考慮
         ↑移動前に当たってるかチェックを入れたからOK
         */
        return SupCapSphereAndSphere(sphere, capsule);
    }
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& capsule,
                          const MoveCollData<SphereCollision>& sphere){
        return MoveCollision(sphere, capsule);
    }
    
    //Capsule and Capsule
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& cap1,
                          const MoveCollData<CapsuleCollision>& cap2){
        
        HitData ret;
        HitData data[3];
        
        //        float t[2];
        //        Vector3 pos[2];
        //        float dist = SupSegmentSegmentDist(cap1.collision.s, cap2.collision.s, t[0], t[1], pos[0], pos[1]);
        //        float radiusSum  = cap1.collision.radius + cap2.collision.radius;
        //        ret.hitPos = (pos[1] - pos[0]) * (cap1.collision.radius / radiusSum) + pos[0];
        //
        //        if(dist < radiusSum){
        //            ret.hit = true;
        //            ret.time = 0.0f;
        //            if(IsParallel(cap1.collision.s.v, cap2.collision.s.v)){
        //                Vector3 hPos = SupParaCylinderHitPoint(CylinderCollision(cap1.collision.radius, cap1.collision.s), CylinderCollision(cap2.collision.radius, cap2.collision.s), pos[0], pos[1]);
        //                if(IsInside(cap1.collision.s, ret.hitPos)){
        //                    ret.hitPos = hPos;
        //                    hPos = CastToLine(cap1.collision.s, ret.hitPos);
        //                    ret.hitNormal = Normalize(hPos - ret.hitPos);
        //                    ret.length = radiusSum - (pos[1] - pos[0]).Length();
        //                    return ret;
        //                }
        //            }
        //            ret.hitNormal = Normalize(pos[1] - pos[0]);
        //            ret.length = radiusSum - (pos[1] - pos[0]).Length();
        //            return ret;
        //        }
        
        Vector3 cap1Pos = cap1.phys.GetPosition();
        Vector3 cap1Vel = CulcVel(cap1.phys) - CulcVel(cap2.phys);
        CylinderCollision cylinder1(cap1.collision.radius, cap1.collision.s);
        CylinderCollision cylinder2(cap2.collision.radius, cap2.collision.s);
        data[0] = MoveCollision(MoveCollData<CylinderCollision>(cylinder1,cap1.phys), MoveCollData<CylinderCollision>(cylinder2,cap2.phys));
        if(data[0].hit){
            Vector3 p1 = cap1Pos + CulcVel(cap1.phys) * data[0].time;
            Vector3 p2 = cap2.phys.GetPosition() + CulcVel(cap2.phys) * data[0].time;
            if(IsInside(Segment(p1, cap1.collision.s.v), data[0].hitPos) &&
               IsInside(Segment(p2, cap2.collision.s.v), data[0].hitPos)){
                return data[0];
            }
            else {
                data[0].hit = false;
            }
            CapsuleCollision sweepedBeginSphere(cap1.collision.radius,cap1Pos,cap1Vel);
            CapsuleCollision sweepedEndSphere(cap1.collision.radius,cap1Pos + cap1.collision.s.v,cap1Vel);
            if(CollisionReturnFlag(sweepedBeginSphere, cap2.collision)){
                data[1] = MoveCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(cap1.collision.radius, cap1Pos),cap1.phys), cap2);
            }
            
            if(CollisionReturnFlag(sweepedEndSphere, cap2.collision)){
                Physics tmp;
                tmp.SetPosition(cap1Pos + cap1.collision.s.v,false);
                tmp.SetPrePos(cap1Pos + cap1Vel + cap1.collision.s.v);
                data[2] = MoveCollision(MoveCollData<SphereCollision>(SphereCollision(cap1.collision.radius,cap1Pos + cap1.collision.s.v),tmp), cap2);
            }
            
            if(data[1].hit){
                if(data[2].hit){
                    //ここに来るのは平行なときぐらいなきがするからその場合cylinderの判定で終わってそうやからいらん気がする
                    if(data[1].time < data[2].time){
                        return data[1];
                    }
                    else {
                        return data[2];
                    }
                }
                else {
                    return data[1];
                }
            }
            else {
                return data[2];
            }
        }
        else {
            return ret;
        }
        
        
        
        
        
        //        CylinderCollision lhs(cap1.collision.radius, cap1.collision.s);
        //        data[0] = MoveCollision(MoveCollData<myTools::CylinderCollision>(lhs,cap1.phys), cap2);
        //        if(data[0].hit){
        //
        //            Vector3 pos = cap1.phys.GetPosition() + CulcVel(cap1.phys) * data[0].time;
        //            data[0].hit = IsInside(Segment(pos, cap1.collision.s.v), data[0].hitPos);
        //
        //            SphereCollision sphereColl(cap1.collision.radius, cap1.collision.s.p);
        //            data[1] = MoveCollision(MoveCollData<SphereCollision>(sphereColl,cap1.phys), cap2);
        //            sphereColl.position = cap1.collision.s.p + cap1.collision.s.v;
        //            Physics phys;
        //            phys.SetPosition(cap1.collision.s.p + cap1.collision.s.v,false);
        //            phys.SetPrePos(cap1.phys.GetPrePos() + cap1.collision.s.v);
        //            data[2] = MoveCollision(MoveCollData<myTools::SphereCollision>(sphereColl,phys), cap2);
        //
        //            for(auto& d : data){
        //                if(d.hit){
        //                    if(!ret.hit){
        //                        ret = d;
        //                    }
        //                    else {
        //                        if(d.time < ret.time){
        //                            ret = d;
        //                        }
        //                    }
        //                }
        //            }
        //        }
        //
        //
        //        return ret;
    }
    
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Point& point){
        HitData ret;
        Vector3 spherePos = sphere.phys.GetPosition();
        float distSq = (spherePos - point).LengthSq();
        float radiusSq = sphere.collision.radius * sphere.collision.radius;
        if(distSq <= radiusSq){
            ret.hit = true;
            ret.hitPos = point;
            ret.hitNormal = Normalize(spherePos - point);
            ret.time = 0.0f;
            ret.length = sphere.collision.radius - sqrtf(distSq);
        }
        
        Vector3 sphereMoved = sphere.phys.GetPrePos();
        Vector3 start = point - spherePos;
        Vector3 end = point - sphereMoved;
        Vector3 vel = end - start;
        float a = vel.LengthSq();
        float b = dot(start, vel);
        float c = start.LengthSq() - radiusSq;
        float ans = b * b - a * c;
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(0.0f <= t && t <= 1.0f){
                ret.hit = true;
                ret.time = t;
                ret.hitPos = point;
                ret.hitNormal = -Normalize(start + t * vel);
            }
            return ret;
        }
        
        ans = sqrtf(ans);
        
        float t[2] = {
            (-b - ans) / a,
            (-b + ans) / a
        };
        
        if(0.0f <= t[0] && t[0] <= 1.0f){
            ret.hit = true;
            ret.time = t[0];
            ret.hitPos = point;
            ret.hitNormal = -Normalize(start + t[0] * vel);
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Line& line){
        HitData ret;
        Vector3 spherePos = sphere.collision.position;
        float radius = sphere.collision.radius;
        if(CollisionReturnFlag(sphere.collision, line)){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = CastToLine(line, spherePos);;
            Vector3 w = spherePos - ret.hitPos;
            ret.hitNormal = Normalize(w);
            ret.length = radius - w.Length();
            return ret;
        }
        
        Vector3 movedPos = sphere.phys.GetPrePos();
        Vector3 start = CastToLine(line, spherePos) - spherePos;
        Vector3 end = CastToLine(line, movedPos) - movedPos;
        Vector3 vel = end - start;
        float a = vel.LengthSq();
        float b = dot(start, vel);
        float c = start.LengthSq() - radius * radius;
        float ans = b * b - a * c;
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        
        if(ans < MT_EPSILON){
            float t = -b / a;
            if(0.0f <= t && t <= 1.0f){
                ret.hit = true;
                ret.time = t;
                Vector3 pos = spherePos + t * (movedPos - spherePos);
                ret.hitPos = CastToLine(line, pos);
                ret.hitNormal = Normalize(pos - ret.hitPos);
            }
            return ret;
        }
        
        ans = sqrtf(ans);
        float t = (-b - ans) / a;
        if(0.0f <= t && t <= 1.0f){
            ret.hit = true;
            ret.time = t;
            Vector3 pos = spherePos + t * (movedPos - spherePos);
            ret.hitPos = CastToLine(line, pos);
            ret.hitNormal = Normalize(pos - ret.hitPos);
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Segment& segment){
        Vector3 spherePos = sphere.collision.position;
        Line segLine(segment);
        HitData ret = StaticCollision(sphere,segLine);
        if(!ret.hit){
            return ret;
        }
        if(IsInside(segment, ret.hitPos)){
            return ret;
        }
        HitData data[2];
        data[0] = StaticCollision(sphere, segLine.p);
        data[1] = StaticCollision(sphere, segLine.p + segLine.v);
        if(data[0].hit){
            if(data[1].hit){
                return data[0].time < data[1].time ? data[0] : data[1];
            }
            else {
                return data[0];
            }
        }
        else {
            return data[1];
        }
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const PlaneCollision& plane){
        HitData ret;
        Vector3 spherePos = sphere.phys.GetPosition();
        
        //面の裏にある場合
        if(!IsFront(plane, spherePos)){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = CastToPlane(plane, spherePos);
            ret.length = (spherePos - ret.hitPos).Length() + sphere.collision.radius;
            ret.hitNormal = plane.normal;
            return ret;
        }
        
        float dist = Distance(sphere.collision.position, plane);
        if(dist < sphere.collision.radius){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = CastToPlane(plane, spherePos);
            ret.length = sphere.collision.radius - dist;
            ret.hitNormal = plane.normal;
            return ret;
        }
        
        //これでいけそう
        Vector3 movedPos = sphere.phys.GetPrePos();
        dist = dot(plane.normal, plane.p - spherePos);
        float movedDist = dot(plane.normal, plane.p - movedPos);
        if(dist < 0){
            dist *= -1;
            movedDist *= -1;
        }
        float vel = movedDist - dist;
        float t = (sphere.collision.radius - dist)/vel;
        if(0.0f <= t && t <= 1.0f){
            ret.hit = true;
            ret.time = t;
            movedPos = sphere.collision.position + (movedPos - sphere.collision.position) * t;
            ret.hitPos = CastToPlane(plane, movedPos);
            ret.hitNormal = plane.normal;
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Line& line){
        HitData ret;
        float t[2];
        Vector3 pos[2];
        float radiusSq = cylinder.collision.radius * cylinder.collision.radius;
        float distSq = SupLineLineDistSq(cylinder.collision.line, line, t[0], t[1], pos[0], pos[1]);
        if(distSq <= radiusSq){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = pos[1];
            Vector3 w = pos[0] - pos[1];
            ret.length = cylinder.collision.radius - w.LengthSq();
            ret.hitNormal = Normalize(w);
            return ret;
        }
        Vector3 cylPos = pos[0];
        Vector3 start = pos[1] - pos[0];
        Line movedLine(cylinder.phys.GetPrePos(), cylinder.collision.line.v);
        SupLineLineDistSq(movedLine, line, t[0], t[1], pos[0], pos[1]);
        Vector3 cylMovedPos = pos[0];
        Vector3 end = pos[1] - pos[0];
        Vector3 vel = end - start;
        float a = vel.LengthSq();
        float b = dot(start, vel);
        float c = start.LengthSq() - radiusSq;
        float ans = b * b - a * c;
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        
        //        if(ans < MT_EPSILON){
        //            float d = -b / a;
        //            if(0.0f <= d && d <= 1.0f){
        //                ret.hit = true;
        //                ret.time = d;
        //                Vector3 cylHitPos = cylPos + (cylMovedPos - cylPos) * d;
        //                Vector3 w = start + vel * d;
        //                ret.hitPos = cylHitPos + w;
        //                ret.hitNormal = Normalize(-w);
        //            }
        //            return ret;
        //        }
        //
        //        ans = sqrtf(ans);
        //        float d = (-b - ans) / a;
        //        if(0.0f <= d && d <= 1.0f){
        //
        //        }
        
        float d = (ans < MT_EPSILON) ? -b / a : (-b - sqrtf(ans)) / a;
        if(0.0f <= d && d <= 1.0f){
            ret.hit = true;
            ret.time = d;
            Vector3 cylHitPos = cylPos + (cylMovedPos - cylPos) * d;
            Vector3 w = start + vel * d;
            ret.hitPos = cylHitPos + w;
            ret.hitNormal = -Normalize(w);
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Point& point){
        HitData ret;
        float t;
        Vector3 pos;
        float distSq = SupPointLineDistSq(point, cylinder.collision.line, pos, t);
        float radius = cylinder.collision.radius;
        if(distSq <= radius * radius){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitPos = point;
            Vector3 w = pos - point;
            ret.length = radius - sqrtf(distSq);
            ret.hitNormal = Normalize(w);
            return ret;
        }
        
        Vector3 start = pos - point;
        SupPointLineDistSq(point, Line(cylinder.phys.GetPrePos(), cylinder.collision.line.v), pos, t);
        Vector3 end = pos - point;
        Vector3 vel = end - start;
        float a = vel.LengthSq();
        float b = dot(start, vel);
        float c = start.LengthSq() - radius * radius;
        float ans = b * b - a * c;
        if(ans < 0 || a == 0.0f){
            return ret;
        }
        
        float d = (ans < MT_EPSILON) ? (-b / a) : ((-b - sqrtf(ans)) / a);
        if(0.0f <= d && d <= 1.0f){
            ret.hit = true;
            ret.time = d;
            ret.hitPos = point;
            ret.hitNormal = Normalize(start + vel * d);
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Segment& segment){
        HitData ret;
        //        float t[2];
        //        Vector3 pos[2];
        //        float distSq = SupLineLineDistSq(cylinder.collision.line, segment, t[0], t[1], pos[0], pos[1]);
        //        if(t[1] < 0.0f){
        //            pos[1] = segment.p;
        //            distSq = DistanceSq(pos[1], cylinder.collision.line);
        //        }
        //        else if(1.0f < t[1]){
        //            pos[1] = segment.p + segment.v;
        //            distSq = DistanceSq(pos[1], cylinder.collision.line);
        //        }
        //
        //        if(distSq <= cylinder.collision.radius * cylinder.collision.radius){
        //            ret.hit = true;
        //            ret.time = 0.0f;
        //            ret.hitPos = pos[1];
        //            ret.length = cylinder.collision.radius - sqrtf(distSq);
        //            ret.hitNormal = Normalize((CastToLine(cylinder.collision.line, pos[1])) - pos[1]);
        //            return ret;
        //        }
        
        ret = StaticCollision(cylinder, Line(segment.p, segment.v));
        if(!ret.hit){
            return ret;
        }
        if(ret.hit){
            if(IsParallel(cylinder.collision.line.v, segment.v)){
                ret.hitPos = segment.p + segment.v * 0.5f;
                return ret;
            }
            ret.hit = IsInside(segment, ret.hitPos);
            if(ret.hit){
                return ret;
            }
        }
        
        Vector3 cylMoved = cylinder.phys.GetPrePos();
        Vector3 segmentEnd = segment.p + segment.v;
        PlaneCollision cylPlane(cylinder.collision.line.p,Normalize(cross(cylinder.collision.line.v,cylMoved -cylinder.collision.line.p)));
        HitData data[2] = {
            (Distance(segment.p, cylPlane) <= cylinder.collision.radius) ? StaticCollision(cylinder, segment.p) : HitData(),
            (Distance(segmentEnd, cylPlane) <= cylinder.collision.radius) ? StaticCollision(cylinder, segmentEnd) : HitData()
        };
        
        if(data[0].hit){
            if(data[1].hit){
                if(data[0].time < data[1].time){
                    return data[0];
                }
                else {
                    return data[1];
                }
            }
            else {
                return data[0];
            }
        }
        else {
            return data[1];
        }
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const PlaneCollision& plane){
        HitData ret;
        float t;
        if(SupPlaneLineColl(plane, cylinder.collision.line, t, ret.hitPos)){
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitNormal = plane.normal;
            return ret;
        }
        
        Vector3 pos = cylinder.collision.line.p;
        
        if(!IsFront(plane, pos)){
            ret.hit = true;
            ret.time = 0.0f;
            Vector3 onPlane = CastToPlane(plane, pos);
            ret.hitPos = onPlane + cylinder.collision.line.v * 0.5f;
            ret.length = cylinder.collision.radius + (onPlane - pos).Length();
            ret.hitNormal = plane.normal;
            return ret;
        }
        
        //TODO : すでに接していて平行の場合の処理かく
        if(Distance(pos,plane) <= cylinder.collision.radius){
            ret.hit = true;
            ret.time = 0.0f;
            Vector3 onPlane = CastToPlane(plane, pos);
            ret.hitPos = onPlane + cylinder.collision.line.v * 0.5f;
            ret.length = cylinder.collision.radius - (onPlane - pos).Length();
            ret.hitNormal = plane.normal;
            return ret;
        }
        
        Vector3 movedPos = cylinder.phys.GetPrePos();
//        Vector3 start = CastToPlane(plane, pos) - pos;
//        Vector3 end = CastToPlane(plane, movedPos) - movedPos;
//        Vector3 vel = end - start;
//        float a = vel.LengthSq();
//        float b = dot(start,vel);
//        float c = start.LengthSq() - cylinder.collision.radius * cylinder.collision.radius;
//        float ans = b * b - a * c;
//        if(ans < 0 || a == 0.0f){
//            return ret;
//        }
//        t = (ans < MT_EPSILON ? -b / a : (-b - sqrtf(ans)) / a);
//        if(0.0f <= t && t <= 1.0f){
//            ret.hit = true;
//            ret.time = t;
//            ret.hitPos = pos + (movedPos - pos) * t + cylinder.collision.line.v * 0.5f - plane.normal * cylinder.collision.radius;
//            ret.hitNormal = plane.normal;
//        }
//        return ret;
        
        float dist = -dot(plane.normal, plane.p - pos);
        float movedDist = -dot(plane.normal, plane.p - movedPos);
        float vel = movedDist - dist;
        t = (cylinder.collision.radius - dist ) / vel;
        if(0.0f <= t && t <= 1.0f){
            ret.hit = true;
            ret.time = t;
            ret.hitPos = pos + (movedPos - pos) * t + cylinder.collision.line.v * 0.5f - plane.normal * cylinder.collision.radius;
            ret.hitNormal = plane.normal;
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const PlaneCollision& plane){
        //面と平行な場合
        if (!CollisionReturnFlag(Line(capsule.collision.s.p,capsule.collision.s.v), plane)) {
            return StaticCollision(MoveCollData<myTools::CylinderCollision>(CylinderCollision(capsule.collision.radius, capsule.collision.s),capsule.phys), plane);
        }
        Vector3 capVel = CulcVel(capsule.phys);
        Vector3 capEndPos = capsule.collision.s.p + capsule.collision.s.v;
        CapsuleCollision sweepBeginStartSphere(capsule.collision.radius,capsule.collision.s.p,capVel);
        CapsuleCollision sweepEndSphere(capsule.collision.radius,capEndPos,capVel);
        HitData data[3];
        if(CollisionReturnFlag(sweepBeginStartSphere, plane)){
            data[0] = StaticCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(capsule.collision.radius,capsule.collision.s.p),capsule.phys), plane);
        }
        if(CollisionReturnFlag(sweepEndSphere, plane)){
            Physics tmp;
            tmp.SetPosition(capEndPos, false);
            tmp.SetPrePos(capEndPos + capVel);
            data[1] = StaticCollision(MoveCollData<SphereCollision>(SphereCollision(capsule.collision.radius, capEndPos),tmp), plane);
        }
        data[2] = StaticCollision(MoveCollData<myTools::CylinderCollision>(CylinderCollision(capsule.collision.radius,capsule.collision.s),capsule.phys), plane);
        if(data[2].hit){
            data[2].hit = IsInside(capsule.collision.s, data[2].hitPos);
        }
        HitData ret;
        for(auto& d : data){
            if(!ret.hit){
                ret = d;
            }
            else{
                if(d.time <= ret.time){
                    ret = d;
                }
            }
        }
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const SquareCollision& square){
        HitData ret;
        Vector3 spherePos = sphere.collision.position;
        Vector3 squareNormal = square.GetNormal();
        Vector3 sphereMovedPos = sphere.phys.GetPrePos();
        PlaneCollision squarePlane(square.p[0], squareNormal);
        
        if(SupSquareSphereColl(square, sphere.collision, ret.hitPos)){
            ret.hit = true;
            ret.time = 0.0f;
            ret.length = sphere.collision.radius - Distance(sphere.collision.position, ret.hitPos);
            ret.hitNormal = squareNormal;
            return ret;
        }
        
        ret = StaticCollision(sphere, squarePlane);
        if(!ret.hit){
            return ret;
        }
        if(ret.hit){
            if(CollisionReturnFlag(square, ret.hitPos)){
                return ret;
            }
            ret.hit = false;
        }
        
        Vector3 sphereVel = sphereMovedPos - spherePos;
        CapsuleCollision sweepSphere(sphere.collision.radius, spherePos,sphereVel);
        
        std::vector<Segment> sides = square.GetSides();
        HitData buf;
        if(CollisionReturnFlag(sweepSphere, sides[0])){
            ret = StaticCollision(sphere, sides[0]);
        }
        if(CollisionReturnFlag(sweepSphere, sides[1])){
            buf = StaticCollision(sphere, sides[1]);
            if(buf.hit){
                if(!ret.hit || buf.time < ret.time){
                    ret = buf;
                }
            }
        }
        if(CollisionReturnFlag(sweepSphere, sides[2])){
            buf = StaticCollision(sphere, sides[2]);
            if(buf.hit){
                if(!ret.hit || buf.time < ret.time){
                    ret = buf;
                }
            }
        }
        if(CollisionReturnFlag(sweepSphere, sides[3])){
            buf = StaticCollision(sphere, sides[3]);
            if(buf.hit){
                if(!ret.hit || buf.time < ret.time){
                    ret = buf;
                }
            }
        }
        
        return ret;
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const SquareCollision& square){
        Vector3 squNormal = square.GetNormal();
        const Line& cylLine = cylinder.collision.line;
        PlaneCollision squPlane(square.p[0],squNormal);
        HitData ret = StaticCollision(cylinder, squPlane);
        if(!ret.hit){
            return ret;
        }
        //面と平行な場合
        if(fabsf(dot(squNormal, cylLine.v)) < MT_EPSILON){
            bool backFlag = false;
            if(dot(squPlane.normal,cylLine.p - squPlane.p) < 0){
                backFlag = true;
                ret = StaticCollision(cylinder, PlaneCollision(squPlane.p, -squPlane.normal));
            }
            //平面と衝突時に衝突線がスクエアと交差していれば衝突点を算出してそのまま返す
            Vector3 cylVel = CulcVel(cylinder.phys);
            Vector3 castLinePos = CastToPlane(squPlane, cylLine.p + cylVel * ret.time);
            Line castLine(castLinePos, cylLine.v);
            std::vector<Segment> sides = square.GetSides();
            float t[2];
            Vector3 buf;
            Vector3 pos[2];
            int posIndex = 0;
            for(int i = 0; i < 4; ++i){
                if(IsParallel(castLine.v, sides[i].v)){
                    continue;
                }
                if(SupLineLineColl(castLine, sides[i], t[0], t[1], buf)){
                    if(0.0f <= t[1] && t[1] <= 1.0f){
                        pos[posIndex] = buf;
                        ++posIndex;
                    }
                }
            }
            if(posIndex != 0){
                ret.hitPos = (pos[0] + pos[1]) * 0.5f;
                return ret;
            }
            else {
                Line onHitLine(cylLine.p + cylVel * ret.time,cylLine.v);
                float tmp;
                float minDist = 100000.0f;
                const Vector3* hitVert[2];
                const std::vector<Point> verts = square.GetPoints();
                for(int i = 0; i < 4; ++i){
                    tmp = Distance(verts[i], onHitLine);
                    if(tmp <= minDist){
                        if(minDist - tmp < MT_EPSILON){
                            hitVert[1] = &verts[i];
                        }
                        else {
                            hitVert[0] = &verts[i];
                            hitVert[1] = nullptr;
                        }
                        minDist = tmp;
                    }
                }
                if(hitVert[1]){
                    return StaticCollision(cylinder, Segment(*hitVert[0], *hitVert[1] - *hitVert[0]));
                }
                else {
                    return StaticCollision(cylinder, *hitVert[0]);
                }
            }
            
        }
        else {
            if(CollisionReturnFlag(square, ret.hitPos)){
                return ret;
            }
            std::vector<Segment> sides = square.GetSides();
            //一番近い線分を探す
            int index = 0;
            float dist;
            float t[2];
            Vector3 pos[2];
            float minDist = 100000.0f;// SupLineLineDistSq(sides[0], cylLine, t[0], t[1], pos[0], pos[1]);//DistanceSq(ret.hitPos, sides[0]);
            for(int i = 0; i < 4; ++i){
                dist = SupLineLineDistSq(cylLine, sides[i], t[0], t[1], pos[0], pos[1]);
                if(t[1] < 0.0f || 1.0f < t[1]){
                    Clamp(t[1], 0.0f, 1.0f);
                    dist = DistanceSq(sides[i].p + sides[i].v * t[1] , cylLine);
                }
                
                //dist = DistanceSq(ret.hitPos, sides[i]);
                if(dist < minDist){
                    minDist = dist;
                    index = i;
                }
            }
            return StaticCollision(cylinder, sides[index]);
        }
    }
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const SquareCollision& square){
        HitData* ret = nullptr;
        float radius = capsule.collision.radius;
        Vector3 capVel = CulcVel(capsule.phys);
        Vector3 capEndPos = capsule.collision.s.p + capsule.collision.s.v;
        CapsuleCollision sweepStartSphere(radius,capsule.collision.s.p,capVel);
        CapsuleCollision sweepEndSphere(radius, capEndPos,capVel);
        PlaneCollision capMovePlane(capsule.collision.s.p,Normalize(cross(capsule.collision.s.v, capVel)));
        HitData sphereHitData[2];
        
        auto CollEndPosSpheres = [&](HitData*& res){
            //端点の球との判定(ここから)
            if(CollisionReturnFlag(sweepStartSphere, square)){
                sphereHitData[0] = StaticCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(radius, capsule.collision.s.p),capsule.phys), square);
            }
            if(CollisionReturnFlag(sweepEndSphere, square)){
                Physics tmp;
                tmp.SetPosition(capEndPos,false);
                tmp.SetPrePos(capEndPos + capVel);
                sphereHitData[1] = StaticCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(radius,capEndPos),tmp), square);
            }
            
            if(sphereHitData[0].hit){
                if(sphereHitData[1].hit){
                    if(sphereHitData[0].time < sphereHitData[1].time){
                        res = &sphereHitData[0];
                    }
                    else {
                        res = &sphereHitData[1];
                    }
                }
                else {
                    res = &sphereHitData[0];
                }
            }
            else if(sphereHitData[1].hit){
                res = &sphereHitData[1];
            }
            else {
                res = nullptr;
            }
            //端点の球との判定(ここまで)
        };

        
        const std::vector<Segment>& sides = square.GetSides();
        MoveCollData<CylinderCollision> capCylinder(CylinderCollision(radius, capsule.collision.s),capsule.phys);
        HitData cylHitdata = StaticCollision(capCylinder, square);
        if(!cylHitdata.hit){
            return cylHitdata;
        }
        //カプセルが面と平行なとき
        if(fabsf(dot(capsule.collision.s.v,square.GetNormal())) < MT_EPSILON){
            //衝突点の算出
            Vector3 onHitPos = capsule.collision.s.p + capVel * cylHitdata.time;
            PlaneCollision squPlane(square.p[0],square.GetNormal());
            if(Distance(onHitPos,squPlane) < capsule.collision.radius){
                //辺か線分と衝突
                Segment castSegment(CastToPlane(squPlane, onHitPos),capsule.collision.s.v);
                Vector3 buf;
                Vector3 lineHitPos[2];
                //辺と当たっているのか頂点と当たっているのか調べる
                //衝突時線との距離を測って一番近い点を探す
                //近い点が２つあればその2点の線分との中点がカプセルのシリンダー内かどうか判定
                const std::vector<Point>& verts = square.GetPoints();
                float dist = 100000.0f;
                float tmp;
                const Point* hitPoints[2] = {nullptr, nullptr};
                Line* castLine(&castSegment);
                for(int i = 0; i < 4; ++i){
                    tmp = DistanceSq(verts[i], *castLine);
                    if(tmp <= dist){
                        if(hitPoints[0] == nullptr){
                            hitPoints[0] = &verts[i];
                        }
                        else {
                            if(dist - tmp < MT_EPSILON){
                                hitPoints[1] = &verts[i];
                            }
                            else {
                                hitPoints[0] = &verts[i];
                                hitPoints[1] = nullptr;
                            }
                        }
                        dist = tmp;
                    }
                }
                //2点当たっている場合
                if(hitPoints[1]){
                    Segment tmp(*hitPoints[0], *hitPoints[1] - *hitPoints[0]);
                    Vector3 pos[2];
                    Vector3 res = SupParaSegCenter(castSegment, tmp, pos[0], pos[1]);
                    if(IsInside(castSegment, pos[0])){
                        cylHitdata.hitPos = CastToLine(tmp, res);
                        return cylHitdata;
                    }
                }
                //1点しか当たっていない場合
                else if(IsInside(castSegment, CastToLine(castSegment, *hitPoints[0]))){
                    return cylHitdata;
                }
                //円と判定
                CollEndPosSpheres(ret);
                if(ret){
                    return *ret;
                }
                else {
                    return HitData();
                }
            }
            else {
                //面と衝突
                Segment castSegment(CastToPlane(squPlane, onHitPos),capsule.collision.s.v);
                int inSideCount = 0;
                if(CollisionReturnFlag(square, castSegment.p)){
                    ++inSideCount;
                }
                if(CollisionReturnFlag(square, castSegment.GetEndPoint())){
                    ++inSideCount;
                }
                if(inSideCount == 2){
                    cylHitdata.hitPos = castSegment.p + castSegment.v * 0.5f;
                    return cylHitdata;
                }
                
                float t[2];
                Vector3 buf;
                Vector3 lineHitPos[2];
                int hitCount = 0;
                bool para = false;
                for(int i = 0; i < 4; ++i){
                    if(SupLineLineColl(castSegment, sides[i], t[0], t[1], buf)){
                        if(IsParallel(castSegment.v, sides[i].v)){
                            para = true;
                            continue;
                        }
                        else {
                            if(0.0f <= t[0] && t[0] <= 1.0f &&
                               0.0f <= t[1] && t[1] <= 1.0f){
                                lineHitPos[hitCount] = buf;
                                ++hitCount;
                            }
                        }
                    }
                }
                switch (hitCount) {
                    case 1:
                        cylHitdata.hitPos = (lineHitPos[0] +
                                             (CollisionReturnFlag(square, castSegment.p) ? castSegment.p: castSegment.p + castSegment.v * 0.5f));
                        return cylHitdata;
                        break;
                    case 2:
                        cylHitdata.hitPos = (lineHitPos[0] + lineHitPos[1]) * 0.5f;
                        return cylHitdata;
                    default:
                        if(para){
                            cylHitdata.hitPos = (castSegment.p + castSegment.v * 0.5f);
                            return cylHitdata;
                        }
                        Physics tmp;
                        tmp.SetPosition(capEndPos,false);
                        tmp.SetPrePos(capEndPos + capVel);
                        HitData data[2] = {
                            StaticCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(capsule.collision.radius, capsule.collision.s.p),capsule.phys), square),
                            StaticCollision(MoveCollData<SphereCollision>(SphereCollision(capsule.collision.radius, capEndPos),tmp), square),
                        };
                        if(data[0].hit){
                            if(data[1].hit){
                                if(data[0].time < data[1].time){
                                    return data[0];
                                }
                                else {
                                    return data[1];
                                }
                            }
                            else {
                                return data[0];
                            }
                        }
                        else {
                            return data[1];
                        }
                        
                        break;
                }
            }
        }
        //シリンダーが面と平行じゃない
        Segment onHitSeg(capsule.collision.s.p + capVel * cylHitdata.time,capsule.collision.s.v);
        if(IsInside(onHitSeg, cylHitdata.hitPos)){
            return cylHitdata;
        }
        
        Physics tmp;
        tmp.SetPosition(capEndPos,false);
        tmp.SetPrePos(capEndPos + capVel);
        HitData data[2] = {
            StaticCollision(MoveCollData<myTools::SphereCollision>(SphereCollision(capsule.collision.radius, capsule.collision.s.p),capsule.phys), square),
            StaticCollision(MoveCollData<SphereCollision>(SphereCollision(capsule.collision.radius, capEndPos),tmp), square),
        };
        if(data[0].hit){
            if(data[1].hit){
                if(data[0].time < data[1].time){
                    return data[0];
                }
                else {
                    return data[1];
                }
            }
            else {
                return data[0];
            }
        }
        else {
            return data[1];
        }
    }
//    {
//        HitData ret;
//        Vector3 squNormal = square.GetNormal();
//        PlaneCollision squPlane(square.p[0],squNormal);
//        if(fabsf(dot(capsule.collision.s.v,squNormal)) < MT_EPSILON){
//            ret = StaticCollision(MoveCollData<myTools::CylinderCollision>(CylinderCollision(capsule.collision.radius,capsule.collision.s),capsule.phys), square);
//            if(!ret.hit){
//                return ret;
//            }
//            //衝突時のカプセル線分をスクエア平面にキャスト
//            Vector3 capHitPos = capsule.collision.s.p + CulcVel(capsule.phys) * ret.time;
//            Segment castSegment(capHitPos,capsule.collision.s.v);
//            //キャストした線分とスクエアの辺と交差しているか判定
//            std::vector<Segment> sides = square.GetSides();
//            int hitCount = -1;
//            Vector3 crossPos;
//            Vector3 hitPos[2];
//            float t[2];
//            for(int i = 0; i < 4; ++i){
//                if(SupLineLineColl(castSegment, sides[i], t[0], t[1], crossPos)){
//                    if(0.0f <= t[0] && t[0] <= 1.0f &&
//                       0.0f <= t[1] && t[1] <= 1.0f){
//                        if(hitCount == -1){
//                            hitCount = i;
//                            hitPos[0] = crossPos;
//                        }
//                        else {
//                            hitCount += i * 10;
//                            hitPos[1] = crossPos;
//                        }
//                    }
//                }
//            }
//
//            if(hitCount >= 0){
//                if(hitCount >= 10){
//                    ret.hitPos = (hitPos[0] + hitPos[1]) * 0.5f;
//                }
//                ret.hitPos = (hitPos[0] +
//                              (CollisionReturnFlag(square, castSegment.p) ? castSegment.p : castSegment.p + castSegment.v)) * 0.5f;
//                return ret;
//            }
//            else {
//                //衝突時線との距離を測って一番近い点を探す
//                //近い点が２つあればその2点の線分との中点がカプセルのシリンダーないかどうか判定
//                const std::vector<Point>& verts = square.GetPoints();
//                float dist = 100000.0f;
//                float tmp;
//                const Point* hitPoints[2] = {nullptr, nullptr};
//                Line* castLine(&castSegment);
//                for(int i = 0; i < 4; ++i){
//                    tmp = DistanceSq(verts[i], *castLine);
//                    if(tmp <= dist){
//                        if(hitPoints[0] == nullptr){
//                            hitPoints[0] = &verts[i];
//                        }
//                        else {
//                            if(dist - tmp < MT_EPSILON){
//                                hitPoints[1] = &verts[i];
//                            }
//                            else {
//                                hitPoints[0] = &verts[i];
//                                hitPoints[1] = nullptr;
//                            }
//                        }
//                    }
//                }
//                if(hitPoints[1]){
//                    Segment tmp(*hitPoints[0], *hitPoints[1] - *hitPoints[0]);
//                    Vector3 pos[2];
//                    Vector3 res = SupParaSegCenter(castSegment, tmp, pos[0], pos[1]);
//                    if(IsInside(castSegment, pos[0])){
//                        ret.hitPos = CastToLine(tmp, res);
//                        return ret;
//                    }
//                }
//                else {
//                    if(IsInside(castSegment, CastToLine(castSegment, *hitPoints[0]))){
//                        return ret;
//                    }
//                }
//                //球との判定
//
//
//            }
//
//        }
//        else {
//            //カプセルと面が平行じゃない
//        }
//        return ret;
//    }
    
    const SquareCollision* SupFindNearSquare(const std::vector<SquareCollision>& squares, const MoveCollData<SphereCollision>& sphere){
        CapsuleCollision sweepCapsule(sphere.collision.radius,sphere.collision.position,CulcVel(sphere.phys));
        std::vector<const SquareCollision*> hitSquares;
        for(int i = 0; i < 6; ++i){
            if(CollisionReturnFlag(sweepCapsule, squares[i])){
                hitSquares.push_back(&squares[i]);
            }
        }
        
        int hitNum = (int)hitSquares.size();
        switch (hitNum) {
            case 0:
                return nullptr;
            case 1:
                return hitSquares[0];
            default:
            {
                float dist = DistanceSq(sphere.collision.position, *hitSquares[0]);
                float tmp;
                for(int i = 1; i < hitNum; ++i){
                    tmp = DistanceSq(sphere.collision.position, *hitSquares[i]);
                    if(tmp < dist){
                        dist = tmp;
                        hitSquares[0] = hitSquares[i];
                    }
                }
                return hitSquares[0];
            }
        }
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const AABBCollision& aabb){
        std::vector<Point> verts = aabb.GetPoints();
        SquareCollision squares[6] = {
            SquareCollision(verts[0], verts[1], verts[2], verts[3],false),
            SquareCollision(verts[3], verts[2], verts[6], verts[7],false),
            SquareCollision(verts[7], verts[6], verts[5], verts[4],false),
            SquareCollision(verts[4], verts[5], verts[1], verts[0],false),
            SquareCollision(verts[4], verts[0], verts[3], verts[7],false),
            SquareCollision(verts[1], verts[5], verts[6], verts[2],false),
        };
        //すでに衝突しているとき
        if(CollisionReturnFlag(aabb, sphere.collision)){
            HitData ret;
            ret.hit = true;
            ret.time = 0.0f;
            
            int nearIndex = 1;
            SquareCollision* nearSquare = &squares[0];
            float dist = DistanceSq(sphere.collision.position, squares[0]);
            float tmp;
            for(int i = 1; i < 6; ++i){
                tmp = DistanceSq(sphere.collision.position, squares[i]);
                if(tmp < dist){
                    dist = tmp;
                    nearIndex = i;
                    nearSquare = &squares[i];
                }
            }
            dist = sqrtf(dist);
            ret.hitNormal = nearSquare->GetNormal();
            ret.hitPos = CastToPlane(PlaneCollision(nearSquare->p[0], ret.hitNormal),sphere.collision.position);
            ret.length = sphere.collision.radius + (CollisionReturnFlag(aabb, sphere.collision.position) ? dist : -dist);
            return ret;
        }
        
        Vector3 spherevel = CulcVel(sphere.phys);
        float d = 0.0f;
        SquareCollision* hitSquarePointer = nullptr;
        Line moveLine(sphere.collision.position,spherevel);
        PlaneCollision ptmp;
        Vector3 squNormal ;
        float t;
        Vector3 pos;
        for(int i = 0; i < 6; ++i){
            squNormal = cross(squares[i].p[1] - squares[i].p[0], squares[i].p[2] - squares[i].p[1]);
            if(dot(squNormal,spherevel) < 0){
                ptmp.p = squares[i].p[0];
                ptmp.normal = squNormal;
                SupPlaneLineColl(ptmp, moveLine, t, pos);
                if(d < t){
                    d = t;
                    hitSquarePointer = &squares[i];
                }
            }
        }
        
        if(hitSquarePointer){
            hitSquarePointer->CulcNormal();
            return StaticCollision(sphere, *hitSquarePointer);
        }
        else {
            return HitData();
        }
    }
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const AABBCollision& aabb);
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const AABBCollision& aabb){
        //あと事前判定してないからそれも
        Vector3 capPos = capsule.collision.s.p;
        Vector3 capEndPos = capPos + capsule.collision.s.v;
        Vector3 capVel = CulcVel(capsule.phys);

        std::vector<Point> verts = aabb.GetPoints();
        std::vector<SquareCollision> squares{
            SquareCollision(verts[0], verts[1], verts[2], verts[3],false),
            SquareCollision(verts[3], verts[2], verts[6], verts[7],false),
            SquareCollision(verts[7], verts[6], verts[5], verts[4],false),
            SquareCollision(verts[4], verts[5], verts[1], verts[0],false),
            SquareCollision(verts[4], verts[0], verts[3], verts[7],false),
            SquareCollision(verts[1], verts[5], verts[6], verts[2],false),
        };
        
        PlaneCollision squPlane;
        float t;
        Vector3 hitPos;

        Line moveLine[2] = {
            {capPos,capVel},
            {capEndPos,capVel}
        };
        
        PlaneCollision ptmp;
        Vector3 squNormal;
        t = 0.0f;
        float d[2] ={ 0.0f, 0.0f};
        float param;
        SquareCollision* lineHitSquare[2] = {nullptr, nullptr};
        auto check = [&](int index, SquareCollision& square)->bool{
            SupPlaneLineColl(ptmp, moveLine[index], param, hitPos);
            if(d[index] < param){
                d[index] = param;
                lineHitSquare[index] = &square;
                return true;
            }
            return false;
        };
        
        HitData ret;
        HitData buf;
        
        for(int i = 0; i < 6; ++i){
            squNormal = cross(squares[i].p[1] - squares[i].p[0], squares[i].p[2] - squares[i].p[1]);
            if(dot(squNormal, capVel) < 0){
                ptmp.p = squares[i].p[0];
                ptmp.normal = squNormal;
                check(0,squares[i]);
                check(1,squares[i]);
            }
        }

        if(lineHitSquare[0] && lineHitSquare[1]){
            if(lineHitSquare[0] == lineHitSquare[1]){
                lineHitSquare[0]->CulcNormal();
                return StaticCollision(capsule, *lineHitSquare[0]);
            }
        }
        
        //ここ両方判定させんでもいけるように考える
        HitData data[2] = {
            lineHitSquare[0] ? StaticCollision(capsule, *lineHitSquare[0]) : HitData(),
            lineHitSquare[1] ? StaticCollision(capsule, *lineHitSquare[1]) : HitData(),
        };
        
        if(data[0].hit){
            if(data[1].hit){
                return data[0].time < data[1].time ? data[0] : data[1];
            }
            else {
                return data[0];
            }
        }
        else {
            return data[1];
        }
    }
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const SphereCollision& staticSphere){
        if(CollisionReturnFlag(sphere.collision, staticSphere)){
            HitData ret;
            ret.hit = true;
            ret.time = 0.0f;
            Vector3 w = sphere.collision.position - staticSphere.position;
            ret.hitNormal = Normalize(w);
            ret.length = staticSphere.radius + sphere.collision.radius - w.Length();
            ret.hitPos = sphere.collision.position - (ret.length * ret.hitNormal);
            return ret;
        }
        float radSum = sphere.collision.radius + staticSphere.radius;
        Vector3 start = staticSphere.position - sphere.collision.position;
        Vector3 end = staticSphere.position - sphere.phys.GetPrePos();
        Vector3 vel = end - start;
        float a = vel.LengthSq();
        float b = dot(start,vel);
        float c = start.LengthSq() - radSum * radSum;
        float ans = b * b - a * c;
        if(ans < 0 || a == 0.0f){
            return HitData();
        }
        
        float t = ans < MT_EPSILON ? (-b / a) : ((-b - sqrtf(ans)) / a);
        if(0.0f <= t && t <= 1.0f){
            HitData ret;
            ret.hit = true;
            ret.time = t;
            Vector3 onHitPos = sphere.collision.position + CulcVel(sphere.phys) * t;
            ret.hitNormal = Normalize(sphere.collision.position - staticSphere.position);
            ret.hitPos = onHitPos - ret.hitNormal * sphere.collision.radius;
            return ret;
        }
        return HitData();
    }
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const DomeCollision& staticSphere);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const DomeCollision& dome){
        SphereCollision minSphere(dome.minRadius, dome.position);
        if(!CollisionReturnFlag(minSphere, sphere.collision.position)){
            return StaticCollision(sphere, SphereCollision(dome.minRadius,dome.position));
        }

        float sphereRadSq = sphere.collision.radius * sphere.collision.radius;
        float domeMinRadSq = dome.minRadius * dome.minRadius;
        float dist = domeMinRadSq - ((sphere.collision.position - dome.position).LengthSq() + sphereRadSq);
        if(dist < 0){
            HitData ret;
            ret.hit = true;
            ret.time = 0.0f;
            ret.hitNormal = Normalize(sphere.collision.position - dome.position);
            ret.length = -dist;
            ret.hitPos = (sphere.collision.radius - ret.length) * (ret.hitNormal);
            return ret;
        }
        Vector3 moved = sphere.phys.GetPrePos();

        float movedDist = domeMinRadSq - ((moved - dome.position).LengthSq() + sphereRadSq);
        if(movedDist <= 0){
            float vel = movedDist - dist;
            float t = -dist / vel;
            if(0.0f <= t && t <= 1.0f){
                HitData ret;
                ret.hit = true;
                ret.time = t;
                Vector3 onHitPos = sphere.collision.position + CulcVel(sphere.phys) * t;
                ret.hitNormal = Normalize(onHitPos - dome.position);
                ret.hitPos = onHitPos + ret.hitNormal * sphere.collision.radius;
                return ret;
            }
        }
        return HitData();
    }
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const DomeCollision& dome);
    
    void CulcDomeFix(float delta, MoveCollData<SphereCollision>& sphere,const DomeCollision& dome){
        HitData data = StaticCollision(sphere, dome);
        if(!data.hit){
            return;
        }
        
        Vector3 spherePos = sphere.collision.position;
        Vector3 sphereVel = CulcVel(sphere.phys);
        Vector3 spherePreVel = sphere.phys.GetPreVel();
        
        Vector3 spherePosRes = spherePos + data.time * sphereVel;
        
        if(data.time < MT_EPSILON){
            spherePosRes += data.hitNormal * data.length;
        }
        
        //衝突面方向に進んでいる場合その方向の速度を消す
        float t = dot(data.hitNormal, spherePreVel);
        if(t < 0){
            spherePreVel -= t * data.hitNormal;
            sphereVel -= dot(data.hitNormal, sphereVel) * data.hitNormal;
            sphere.phys.AddRestrictVector(data.hitNormal);
        }
        
        if(!CollisionReturnFlag(SphereCollision(dome.minRadius, dome.position), sphere.collision.position)){
            spherePosRes += (1 - data.time) * sphereVel;
            sphere.phys.SetPrePos(spherePosRes);
            sphere.phys.SetPreVel(spherePreVel);
        }
        else {
            float angVel = sphereVel.Length() / (dome.minRadius - sphere.collision.radius);
            Vector3 toSphere = spherePosRes - dome.position;
            Quaternion rotator = MakeQuat(Normalize(cross(spherePosRes - dome.position, sphereVel)), angVel * (1 - data.time));
            toSphere = rotator.rotate(toSphere);
            spherePosRes = toSphere + dome.position;
            sphere.phys.SetPrePos(spherePosRes);
            sphere.phys.SetPreVel(spherePreVel);
        }
    }
    
} // namespace myTools

