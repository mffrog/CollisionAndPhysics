//
//  Camera.h
//  3DCollision
//
//  Created by Tomoya Fujii on 2018/01/12.
//  Copyright © 2018年 TomoyaFujii. All rights reserved.
//

#ifndef Camera_h
#define Camera_h

#include "Vector.h"
#include "Matrix.h"

namespace myTools {
    class Camera {
    public:
        
        void SetPositoin(const Vector3& pos){
            position = pos;
        }
        void SetDirection(const Vector3& dire){
            direction = dire;
        }
        void SetUpVector(const Vector3& upV){
            upVector = upV;
        }
        Vector3 GetPosition(){
            return position;
        }
        Vector3 GetDirection(){
            return direction;
        }
        Vector3 GetUpVector(){
            return upVector;
        }
    private:
        Vector3 position;
        Vector3 direction;
        Vector3 upVector;
    };
}// namespace myTools

#endif /* Camera_h */
