//
//  Physics.h
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/16.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#ifndef Physics_h
#define Physics_h

#include "Vector.h"
#include "Collision.h"
#include <iostream>
#include <math.h>

namespace myTools {
    
    class Physics{
    public:
        void Update(float delta, bool isAccelReset);
        
        Vector3 GetNewPosition(float delta);
        Vector3 GetNewVelocity(float delta);
        
        /**
         *  @tips   If isSmooth is false, velocity and acceleration is reseted
         */
        void SetPosition(const Vector3& position, bool isSmooth = true);
        
        void SetVelocity(const Vector3& velocity);
        
        void SetAcceleration(const Vector3& acceleration);
        
        void SetMass(const float& mass){
            this->mass = mass;
            massRate = 1 / mass;
        }
        
        /**
         *  @tips   If isSmooth is false, velocity and acceleration is reseted
         */
        void AddPosition(const Vector3& difPos, bool isSmooth = true);
        
        void AddVelocity(const Vector3& difVel);
        
        void AddAcceleration(const Vector3& difAcc);
        
        void AddForce(const Vector3& force);
        void AddImpulse(const Vector3& impulse);
        
        
        Vector3 GetPosition() const {
            return position;
        }
        Vector3 GetVelocity() const {
            return velocity;
        }
        Vector3 GetAcceleration() const {
            return acceleration;
        }
        float GetMass() const {
            return mass;
        }
        float GetMassRate() const {
            return massRate;
        }
        void SetPrePos(const Vector3& pos){
            prePos = pos;
        }
        void SetPreVel(const Vector3& vel){
            preVel = vel;
        }
        Vector3 GetPrePos() const {
            return prePos;
        }
        
        Vector3 GetPreVel() const {
            return preVel;
        }
        
        void AddFix(const Vector3& posFix, const Vector3& velFix){
            posSum += posFix.LengthSq();
            posFixes.push_back(posFix);
            velSum += velFix.LengthSq();
            velFixes.push_back(velFix);
        }
        
        void PreFix();
        void Fix();
        
        void ResetFix(){
            posFixes.clear();
            velFixes.clear();

            posSum = 0.0f;
            velSum = 0.0f;
        }
        
        void AddRestrictVector(const Vector3& vec){
            restrictVectores.push_back(vec);
        }
        
        Vector3 CulcRestrictPower(const Vector3& impulse);
    private:
        static float maxVelocity;
        
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        
        Vector3 prePos;
        Vector3 preVel;
        float mass = 1.0f;
        float massRate = 1.0f;
        std::vector<Vector3> posFixes;
        std::vector<Vector3> velFixes;
        
        float posSum;
        float velSum;
        
        std::vector<Vector3> restrictVectores;
    };
    
    //MoveObjects Collisions
    struct HitData{
        bool hit        = false;
        float time      = 0.0f;
        float length    = 0.0f;
        Vector3 hitPos;
        Vector3 hitNormal;
    };
    
    template<typename Ty>
    struct MoveCollData{
        MoveCollData() = default;
        MoveCollData(const Ty& coll, const Physics& phys) : collision(coll), phys(phys){}
        Ty collision;
        Physics phys;
        AABBCollision aabb;
    };
    
    Vector3 CulcVel(const Physics& phys);
    
    //Sphere and Sphere HitTime
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere1,
                          const MoveCollData<SphereCollision>& sphere2);
    
    //Cylinder and Cylinder HitTime
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder1,
                          const MoveCollData<CylinderCollision>& cylinder2);
    
    //Sphere and Cylinder HitTime
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere,
                          const MoveCollData<CylinderCollision>& cylinder);
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder,
                          const MoveCollData<SphereCollision>& sphere);
    
    //Cylinder and Capsule HitTime
    HitData MoveCollision(const MoveCollData<CylinderCollision>& cylinder,
                          const MoveCollData<CapsuleCollision>& capsule);
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& capsule,
                          const MoveCollData<CylinderCollision>& cylinder);
    
    //Sphere and Capsule HitTime
    HitData MoveCollision(const MoveCollData<SphereCollision>& sphere,
                          const MoveCollData<CapsuleCollision>& capsule);
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& capsule,
                          const MoveCollData<SphereCollision>& sphere);
    
    //Capsule and Capsule
    HitData MoveCollision(const MoveCollData<CapsuleCollision>& cap1,
                          const MoveCollData<CapsuleCollision>& cap2);
    
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Point& point);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Line& line);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const Segment& segment);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const PlaneCollision& plane);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Point& point);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Line& line);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const Segment& segment);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const PlaneCollision& plane);
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const PlaneCollision& plane);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const SquareCollision& square);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const SquareCollision& square);
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const SquareCollision& square);

    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const AABBCollision& aabb);
    
    HitData StaticCollision(const MoveCollData<CylinderCollision>& cylinder,
                            const AABBCollision& aabb);
    
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const AABBCollision& aabb);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const SphereCollision& staticSphere);
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const DomeCollision& staticSphere);
    
    HitData StaticCollision(const MoveCollData<SphereCollision>& sphere,
                            const DomeCollision& dome);
    HitData StaticCollision(const MoveCollData<CapsuleCollision>& capsule,
                            const DomeCollision& dome);
    
    template<typename Ty1, typename Ty2>
    void CulcFix(float delta, Ty1& lhs, Ty2& rhs){
        HitData data = MoveCollision(lhs, rhs);
        if(!data.hit){
            return;
        }
        //すでにめり込んでいる時に本来は加わるはずのない方向への力が加わって爆発してるのでそこをまず直す
        //重心から衝突点方向のベクトルとの内積が負の場合本来あたらんはず
        //restrictvectorの伝播ができれば荒ぶりを止めれる
        
        Vector3 v1 = lhs.phys.GetPreVel();
        Vector3 v2 = rhs.phys.GetPreVel();

        Vector3 lhsPos = lhs.phys.GetPosition();
        Vector3 rhsPos = rhs.phys.GetPosition();
        
        Vector3 lhsVel = CulcVel(lhs.phys);
        Vector3 rhsVel = CulcVel(rhs.phys);

        float hitTime = delta * data.time;
        //力の加わる方向
        Vector3 lhsToCenter = Normalize(data.hitPos - (lhsPos + lhsVel * data.time + lhs.collision.GetToCenter()));
        Vector3 rhsToCenter = Normalize(data.hitPos - (rhsPos + rhsVel * data.time + rhs.collision.GetToCenter()));

        if(lhsToCenter == 0.0f){
            lhsToCenter = -rhsToCenter;
        }
        if(rhsToCenter == 0.0f){
            rhsToCenter = -lhsToCenter;
        }
        
        //反発係数
        float t = 0.1f;
        
        Vector3 normedV1 = dot(data.hitNormal, v1) * data.hitNormal;
        Vector3 normedV2 = dot(data.hitNormal, v2) * data.hitNormal;

        Vector3 lhsImpulse;
        Vector3 rhsImpulse;
        
        Vector3 lhsSpringPower;
        Vector3 rhsSpringPower;
        
        Vector3 lhsSpringFix;
        Vector3 rhsSpringFix;
        if(data.time == 0.0f){
            float spring = 100.0f;
            Vector3 springPower = spring * data.length * data.hitNormal;
            float tmp = dot(springPower, lhsToCenter);
            //lhsSpringPower = (tmp > 0 ? -tmp : tmp) * lhsToCenter;
            lhsImpulse = (tmp > 0 ? -tmp : tmp) * lhsToCenter;
            tmp = dot(springPower, rhsToCenter);
            //rhsSpringPower = (tmp > 0 ? -tmp : tmp) * rhsToCenter;
            rhsImpulse = (tmp > 0 ? -tmp : tmp) * rhsToCenter;
            
            Vector3 sinkingVec = data.length * data.hitNormal;
            tmp = dot(sinkingVec, lhsToCenter);
            if(tmp > 0){
                lhsSpringFix = -sinkingVec * 0.5f;
                rhsSpringFix = sinkingVec * 0.5f;
            }
            else {
                lhsSpringFix = sinkingVec * 0.5f;
                rhsSpringFix = -sinkingVec * 0.5f;
            }
            //lhsImpulse = -spring * data.length * lhsToCenter;
            //rhsImpulse = -spring * data.length * rhsToCenter;
            
            //lhsSpringPower = -spring * data.length * lhsToCenter;
            //rhsSpringPower = -spring * data.length * rhsToCenter;
            
            if(dot(normedV1, lhsToCenter) < 0){
                normedV1.x = normedV1.y = normedV1.z = 0;
            }
            if(dot(normedV2, rhsToCenter) < 0){
                normedV2.x = normedV2.y = normedV2.z = 0;
            }
        }
        
        float lhsMass = lhs.phys.GetMass();
        float rhsMass = rhs.phys.GetMass();
        float lhsMassRate = lhs.phys.GetMassRate();
        float rhsMassRate = rhs.phys.GetMassRate();
        
        Vector3 impulse = (1 + t) * (lhsMass * rhsMass) / (lhsMass + rhsMass) * (normedV2 - normedV1);
        
        lhsImpulse += dot( impulse, lhsToCenter) * lhsToCenter;
        rhsImpulse += dot(-impulse, rhsToCenter) * rhsToCenter;

        
        float buf = 1.0f;
        Vector3 lhsRestrict = lhs.phys.CulcRestrictPower(lhsImpulse) * buf;
        Vector3 rhsRestrict = rhs.phys.CulcRestrictPower(rhsImpulse) * buf;

        lhsImpulse += lhsRestrict + rhsRestrict ;
        rhsImpulse += rhsRestrict + lhsRestrict ;
        
        v1 += lhsImpulse * lhsMassRate;
        v2 += rhsImpulse * rhsMassRate;
        
        lhs.phys.AddFix(v1 * (delta - hitTime) + lhsVel * hitTime + lhsPos - lhs.phys.GetPrePos() + lhsSpringFix, lhsImpulse * lhsMassRate + lhsSpringPower);
        rhs.phys.AddFix(v2 * (delta - hitTime) + rhsVel * hitTime + rhsPos - rhs.phys.GetPrePos() + rhsSpringFix, rhsImpulse * rhsMassRate + rhsSpringPower);
    }
    
    //衝突方向の速度は消す
    template<typename Ty1, typename Ty2>
    void CulcMapFix(float delta,Ty1& lhs,const Ty2& rhs, int index = 0, int index2 = 0, int frame = 0){
        if( index == 14 && index2 == 4 && frame > 1535){
            std::cout << "check items" << frame << std::endl;
        }
        HitData data = StaticCollision(lhs, rhs);
        StaticCollision(lhs, rhs);
        StaticCollision(lhs, rhs);
        if(!data.hit){
            return;
        }
        Vector3 lhsPos = lhs.phys.GetPosition();
        Vector3 lhsVel = CulcVel(lhs.phys);
        Vector3 lhsPreVel = lhs.phys.GetPreVel();
        
        Vector3 lhsPosRes = lhsPos + data.time * lhsVel;
        
        if(data.time < MT_EPSILON){
            lhsPosRes += data.hitNormal * data.length;
        }
        
        //衝突面方向に進んでいる場合その方向の速度を消す
        //法線の向きを壁からの向きに定義していないのにやってたからバグってた
        //Staticとの判定は法線方向はStaticからの向きとする
        float t = dot(data.hitNormal, lhsPreVel);
        float reflection = 2.0f;
        if(t < 0){
            lhsPreVel -= t * data.hitNormal * reflection;
            lhsVel -= dot(data.hitNormal, lhsVel) * data.hitNormal * reflection;
            lhs.phys.AddRestrictVector(data.hitNormal);
        }

        lhsPosRes += (1 - data.time) * lhsVel;
        lhs.phys.SetPrePos(lhsPosRes);
        lhs.phys.SetPreVel(lhsPreVel);
    }
    
    void CulcDomeFix(float delta, MoveCollData<SphereCollision>& sphere,const DomeCollision& dome);
}
#endif /* Physics_h */

