   //
//  PrimitiveMesh.cpp
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/12.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#include "PrimitiveMesh.h"
#include "Quaternion.h"
#include "Transform.h"
#include <math.h>
namespace myTools {
    
    std::vector<Vertex> PrimitiveMesh::Update(){
        std::vector<Vertex> vertex(VertexNum());
        for(int i = 0; i < VertexNum(); ++i){
            vertex[i].position = vert[i];
            vertex[i].color = color;
        }
        return vertex;
    }
    
    //Line
    std::vector<GLuint> LineMesh::index;
    unsigned int LineMesh::vertexNum;
    
    LineMesh::LineMesh(){
        vertexNum = 2;
        vert.resize(vertexNum);
        color = {1,1,1,1};
    }
    LineMesh::LineMesh(const Vector3& p1, const Vector3& p2){
        vertexNum = 2;
        vert.resize(vertexNum);
        vert[0] = p1;
        vert[1] = p2;
        color = {1,1,1,1};
    }
    void LineMesh::SetPoint(const Vector3& p1, const Vector3& p2){
        vert[0] = p1;
        vert[1] = p2;
    }
    void LineMesh::SetPoint(unsigned int index, const Vector3& p){
        if(index < vertexNum){
            vert[index] = p;
        }
    }
    
    std::vector<GLuint>& LineMesh::LineDrawMode() {
        if(index.size() != 2){
            index.resize(2);
            index[0] = 0;
            index[1] = 1;
        }
        return index;
    }
    
    std::vector<GLuint>& LineMesh::SurfaceDrawMode() {
        if(index.size() != 3){
            index.resize(3);
            index[0] = 0;
            index[1] = 1;
            index[2] = 0;
        }
        return index;
    }
    
    //Triangle
    std::vector<GLuint> Triangle::index;
    unsigned int Triangle::vertexNum;
    
    Triangle::Triangle(){
        vertexNum = 3;
        vert.resize(vertexNum);
        color = {1,1,1,1};
    }
    Triangle::Triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3){
        vertexNum = 3;
        vert.resize(vertexNum);
        vert[0] = p1;
        vert[1] = p2;
        vert[2] = p3;
        color = {1,1,1,1};
    }
    void Triangle::SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3){
        vert[0] = p1;
        vert[1] = p2;
        vert[2] = p3;
    }
    void Triangle::SetPoint(unsigned int index, const Vector3& p){
        if(index < vertexNum){
            vert[index] = p;
        }
    }
    
    std::vector<GLuint>& Triangle::LineDrawMode() {
        if(index.size() != 6){
            index.resize(6);
            index[0] = 0;
            index[1] = 1;
            index[2] = 1;
            index[3] = 2;
            index[4] = 2;
            index[5] = 0;
        }
        return index;
    }
    
    std::vector<GLuint>& Triangle::SurfaceDrawMode() {
        if(index.size() != 3){
            index.resize(3);
            index[0] = 0;
            index[1] = 1;
            index[2] = 2;
        }
        return index;
    }
    
    //Square
    std::vector<GLuint> Square::index;
    unsigned int Square::vertexNum;
    
    Square::Square(){
        vertexNum = 4;
        vert.resize(vertexNum);
        color = {1,1,1,1};
    }
    Square::Square(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4){
        vertexNum = 4;
        vert.resize(vertexNum);
        vert[0] = p1;
        vert[1] = p2;
        vert[2] = p3;
        vert[3] = p4;
        color = {1,1,1,1};
    }
    void Square::SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4){
        vert[0] = p1;
        vert[1] = p2;
        vert[2] = p3;
        vert[3] = p4;
    }
    void Square::SetPoint(unsigned int index, const Vector3& p){
        if(index < vertexNum){
            vert[index] = p;
        }
    }
    
    std::vector<GLuint>& Square::LineDrawMode() {
        if(index.size() != 8){
            index.resize(8);
            index[0] = 0;
            index[1] = 1;
            index[2] = 1;
            index[3] = 2;
            index[4] = 2;
            index[5] = 3;
            index[6] = 3;
            index[7] = 0;
        }
        return index;
    }
    
    std::vector<GLuint>& Square::SurfaceDrawMode() {
        if(index.size() != 6){
            index.resize(6);
            index[0] = 0;
            index[1] = 1;
            index[2] = 2;
            index[3] = 2;
            index[4] = 3;
            index[5] = 0;
        }
        return index;
    }
    
    //Sphere
    Sphere::Sphere(unsigned int divideNum){
        SetDivideNum(divideNum);
        color = {1.0,1.0,1.0,1.0};
    }
    
    void Sphere::SetRadius(float radius){
        this->radius = radius;
        SetDivideNum(divideNum);
    }
    void Sphere::SetDivideNum(int num){
        divideNum = num;
        int vertNum = (divideNum * 4) * (divideNum * 2 - 1) + 2 ;
        vert.resize(vertNum);
        topIndex = vertNum - 2;
        bottomIndex = vertNum - 1;
        Vector3 v(radius,0.0f,0.0f);
        float rad = M_PI / (divideNum * 2);
        Quaternion quatZDef = MakeQuat(Vector3(0.0f,0.0f,-1.0f), -rad * (divideNum - 1));
        Quaternion quatY = MakeQuat(Vector3(0.0f,1.0f,0.0f), rad);
        v = quatZDef.rotate(v);
        Quaternion quatZ = MakeQuat(Vector3(0.0f,0.0f,-1.0f), rad);
        int i, j;
        
        for(i = 0; i < divideNum * 2 - 1; ++i){
            for(j = 0; j < divideNum * 4; ++j){
                vert[i * (divideNum * 4) + j] = v;
                v = quatY.rotate(v);
            }
            //TODO: 気が向いたら直す
            //v = quatY.rotate(v);
            v = Vector3(radius,0.0f,0.0f);
            v = quatZDef.rotate(v);
            for(int k = 0; k <= i; ++k){
                v = quatZ.rotate(v);
            }
        }
        vert[topIndex] = Vector3(0.0f, radius, 0.0f);
        vert[bottomIndex] = Vector3(0.0f, -radius, 0.0f);
    }
    
    std::vector<Vertex> Sphere::Update(){
        std::vector<Vertex> vertex(vert.size());
        for(int i = 0; i < vert.size(); ++i){
            vertex[i].position = vert[i] + position;
            vertex[i].color = color;
        }
        return vertex;
    }
    
    std::vector<GLuint>& Sphere::LineDrawMode() {
        int squareNum = (divideNum * (divideNum - 1) * 4 * 2);
        int triangleNum = (divideNum * 4 * 2);
        int VInCyrcle = divideNum * 4;
        index.resize(squareNum * 8 + triangleNum * 6);
        for(int i = 0; i < squareNum; ++i){
            index[i * 8 + 0] = i;
            index[i * 8 + 1] = i + VInCyrcle;
            index[i * 8 + 2] = i + VInCyrcle;
            
            if( (i + 1) % VInCyrcle != 0){
                index[i * 8 + 3] = i + VInCyrcle + 1;
                index[i * 8 + 4] = i + VInCyrcle + 1;
                index[i * 8 + 5] = i + 1;
                index[i * 8 + 6] = i + 1;
            }
            else {
                index[i * 8 + 3] = i + 1;
                index[i * 8 + 4] = i + 1;
                index[i * 8 + 5] = i + 1 - VInCyrcle;
                index[i * 8 + 6] = i + 1 - VInCyrcle;
            }
            index[i * 8 + 7] = i;
        }
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 8 + i * 6 + 0] = topIndex;
            index[squareNum * 8 + i * 6 + 1] = i;
            index[squareNum * 8 + i * 6 + 2] = i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 8 + i * 6 + 3] = i + 1;
                index[squareNum * 8 + i * 6 + 4] = i + 1;
            }
            else {
                index[squareNum * 8 + i * 6 + 3] = i + 1 - VInCyrcle;
                index[squareNum * 8 + i * 6 + 4] = i + 1 - VInCyrcle;
            }
            index[squareNum * 8 + i * 6 + 5] = topIndex;
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 0] = bottomIndex;
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 1] = topIndex - VInCyrcle + i;
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 2] = topIndex - VInCyrcle + i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 3] = topIndex - VInCyrcle + i + 1;
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 4] = topIndex - VInCyrcle + i + 1;
            }
            else {
                //TODO: 要チェック
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 3] = topIndex - VInCyrcle;
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 4] = topIndex - VInCyrcle;
            }
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 5] = bottomIndex;
        }
        return index;
    }
    
    std::vector<GLuint>& Sphere::SurfaceDrawMode() {
        int squareNum = (divideNum * (divideNum - 1) * 4 * 2);
        int triangleNum = (divideNum * 4 * 2);
        int VInCyrcle = divideNum * 4;
        index.resize(squareNum * 6 + triangleNum * 3);
        for(int i = 0; i < squareNum; ++i){
            index[i * 6 + 0] = i;
            index[i * 6 + 1] = i + VInCyrcle;
            if( (i + 1 ) % VInCyrcle != 0){
                index[i * 6 + 2] = i + VInCyrcle + 1;
                index[i * 6 + 3] = i + VInCyrcle + 1;
                index[i * 6 + 4] = i + 1;
            }
            else{
                index[i * 6 + 2] = i + 1;
                index[i * 6 + 3] = i + 1;
                index[i * 6 + 4] = i + 1 - VInCyrcle;
            }
            index[i * 6 + 5] = i;
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 6 + i * 3 + 0] = topIndex;
            index[squareNum * 6 + i * 3 + 1] = i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 6 + i * 3 + 2] = i + 1;
            }
            else{
                index[squareNum * 6 + i * 3 + 2] = i + 1 - VInCyrcle;
            }
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 0] = topIndex - VInCyrcle + i;
            index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 1] = bottomIndex;
            if( ( i + 1) % VInCyrcle != 0){
                index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 2] = topIndex - VInCyrcle + i + 1;
            }
            else {
                //TODO: 要チェック
                index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 2] = topIndex - VInCyrcle;
            }
        }
        return index;
    }
    
    //Capsule
    CapsuleMesh::CapsuleMesh(unsigned int divideNum){
        SetDivideNum(divideNum);
        color = {1.0,1.0,1.0,1.0};
    }
    void CapsuleMesh::SetPosition(const Vector3& pos){
        segment.p = pos;
    }
    void CapsuleMesh::SetLength(const Vector3& length){
        segment.v = length;
    }
    void CapsuleMesh::SetRadius(const float &radius){
        this->radius = radius;
        SetDivideNum(divideNum);
    }
    void CapsuleMesh::SetDivideNum(unsigned int num){
        divideNum = num;
        int vertNum = divideNum * divideNum * 4 * 2 + 2;
        int vInCyrcle = divideNum * 4;
        vert.resize(vertNum);
        sphereHalfIndex = divideNum * divideNum * 4;
        float rad = M_PI / (divideNum * 2);
        Vector3 v(radius,0.0f,0.0f);
        Quaternion quatZDef = MakeQuat(Vector3(0.0f,0.0f,1.0f), rad * (divideNum - 1));
        Quaternion quatZ = MakeQuat(Vector3(0.0f,0.0f,1.0f), -rad);
        Quaternion quatY = MakeQuat(Vector3(0.0f,1.0f,0.0f), rad);
        v = quatZDef.rotate(v);
        for(int i = 0; i < divideNum; ++i){
            for(int j = 0; j < vInCyrcle; ++j){
                vert[i * vInCyrcle + j] = v;
                v = quatY.rotate(v);
            }
            v = quatZDef.rotate(Vector3(radius, 0.0f, 0.0f));
            for(int k = 0; k <= i; ++k){
                v = quatZ.rotate(v);
            }
        }
        v = Vector3(radius, 0.0f, 0.0f);
        for(int i = 0; i < divideNum; ++i){
            for(int j = 0; j < vInCyrcle; ++j){
                vert[i * vInCyrcle + j + sphereHalfIndex] = v;
                v = quatY.rotate(v);
            }
            v = Vector3(radius, 0.0f, 0.0f);
            for(int k = 0; k <= i ; ++k){
                v = quatZ.rotate(v);
            }
        }
        
        topIndex = vertNum - 2;
        bottomIndex = vertNum - 1;
        vert[topIndex] = Vector3(0.0f,radius,0.0f);
        vert[bottomIndex] = Vector3(0.0f,-radius,0.0f);
    }
    std::vector<Vertex> CapsuleMesh::Update() {
        std::vector<Vertex> vertex;
        vertex.resize(vert.size());
        float len = segment.v.Length();
        Quaternion quat = MakeQuatVectorToVector(Vector3(0.0f,1.0f,0.0f), Normalize(segment.v));
        for(int i = 0; i < vert.size(); ++i){
            if(i < sphereHalfIndex){
                vertex[i].position.x = vert[i].x;
                vertex[i].position.y = vert[i].y + len;
                vertex[i].position.z = vert[i].z;
            }
            else{
                vertex[i].position = vert[i];
            }
            if(i == topIndex){
                vertex[i].position.y += len;
            }
            vertex[i].position = quat.rotate(vertex[i].position);
            vertex[i].position += segment.p;
            vertex[i].color = color;
        }
        return vertex;
    }
    unsigned int CapsuleMesh::VertexNum() {
        //TODO: 後で直す
        return divideNum * divideNum * 4 * 2 + 2;
    }
    std::vector<GLuint>& CapsuleMesh::LineDrawMode() {
        int squareNum = (divideNum * (divideNum - 1) * 4 * 2 + divideNum * 4);
        int triangleNum = (divideNum * 4 * 2);
        int VInCyrcle = divideNum * 4;
        index.resize(squareNum * 8 + triangleNum * 6);
        for(int i = 0; i < squareNum; ++i){
            index[i * 8 + 0] = i;
            index[i * 8 + 1] = i + VInCyrcle;
            index[i * 8 + 2] = i + VInCyrcle;
            
            if( (i + 1) % VInCyrcle != 0){
                index[i * 8 + 3] = i + VInCyrcle + 1;
                index[i * 8 + 4] = i + VInCyrcle + 1;
                index[i * 8 + 5] = i + 1;
                index[i * 8 + 6] = i + 1;
            }
            else {
                index[i * 8 + 3] = i + 1;
                index[i * 8 + 4] = i + 1;
                index[i * 8 + 5] = i + 1 - VInCyrcle;
                index[i * 8 + 6] = i + 1 - VInCyrcle;
            }
            index[i * 8 + 7] = i;
        }
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 8 + i * 6 + 0] = topIndex;
            index[squareNum * 8 + i * 6 + 1] = i;
            index[squareNum * 8 + i * 6 + 2] = i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 8 + i * 6 + 3] = i + 1;
                index[squareNum * 8 + i * 6 + 4] = i + 1;
            }
            else {
                index[squareNum * 8 + i * 6 + 3] = i + 1 - VInCyrcle;
                index[squareNum * 8 + i * 6 + 4] = i + 1 - VInCyrcle;
            }
            index[squareNum * 8 + i * 6 + 5] = topIndex;
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 0] = bottomIndex;
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 1] = topIndex - VInCyrcle + i;
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 2] = topIndex - VInCyrcle + i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 3] = topIndex - VInCyrcle + i + 1;
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 4] = topIndex - VInCyrcle + i + 1;
            }
            else {
                //TODO: 要チェック
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 3] = topIndex - VInCyrcle;
                index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 4] = topIndex - VInCyrcle;
            }
            index[squareNum * 8 + VInCyrcle * 6 + i * 6 + 5] = bottomIndex;
        }
        return index;
    }
    std::vector<GLuint>& CapsuleMesh::SurfaceDrawMode() {
        int squareNum = (divideNum * (divideNum - 1) * 4 * 2 + divideNum * 4);
        int triangleNum = (divideNum * 4 * 2);
        int VInCyrcle = divideNum * 4;
        index.resize(squareNum * 6 + triangleNum * 3);
        for(int i = 0; i < squareNum; ++i){
            index[i * 6 + 0] = i;
            index[i * 6 + 1] = i + VInCyrcle;
            if( (i + 1 ) % VInCyrcle != 0){
                index[i * 6 + 2] = i + VInCyrcle + 1;
                index[i * 6 + 3] = i + VInCyrcle + 1;
                index[i * 6 + 4] = i + 1;
            }
            else{
                index[i * 6 + 2] = i + 1;
                index[i * 6 + 3] = i + 1;
                index[i * 6 + 4] = i + 1 - VInCyrcle;
            }
            index[i * 6 + 5] = i;
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 6 + i * 3 + 0] = topIndex;
            index[squareNum * 6 + i * 3 + 1] = i;
            if( (i + 1 ) % VInCyrcle != 0){
                index[squareNum * 6 + i * 3 + 2] = i + 1;
            }
            else{
                index[squareNum * 6 + i * 3 + 2] = i + 1 - VInCyrcle;
            }
        }
        
        for(int i = 0; i < VInCyrcle; ++i){
            index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 0] = topIndex - VInCyrcle + i;
            index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 1] = bottomIndex;
            if( ( i + 1) % VInCyrcle != 0){
                index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 2] = topIndex - VInCyrcle + i + 1;
            }
            else {
                //TODO: 要チェック
                index[squareNum * 6 + VInCyrcle * 3 + i * 3 + 2] = topIndex - VInCyrcle;
            }
        }
        return index;
    }
    //Cube
    std::vector<GLuint> Cube::index;
    unsigned int Cube::vertexNum;
    
    Cube::Cube(){
        vertexNum = 8;
        color = {1,1,1,1};
        scale = {1,1,1};
    }
    
    std::vector<Vertex> Cube::Update(){
        std::vector<Vertex> vertex(8);
        vertex[0].position = Vector3(-scale.x, scale.y, scale.z) + position;
        vertex[1].position = Vector3(-scale.x,-scale.y, scale.z) + position;
        vertex[2].position = Vector3( scale.x,-scale.y, scale.z) + position;
        vertex[3].position = Vector3( scale.x, scale.y, scale.z) + position;
        vertex[4].position = Vector3( scale.x, scale.y,-scale.z) + position;
        vertex[5].position = Vector3( scale.x,-scale.y,-scale.z) + position;
        vertex[6].position = Vector3(-scale.x,-scale.y,-scale.z) + position;
        vertex[7].position = Vector3(-scale.x, scale.y,-scale.z) + position;
        for(auto& v : vertex){
            v.color = color;
        }
        return vertex;
    }
    
    std::vector<GLuint>& Cube::LineDrawMode(){
        if(index.size() != 24){
            index.resize(24);
            index[0] = 0;   index[1] = 1;
            index[2] = 1;   index[3] = 2;
            index[4] = 2;   index[5] = 3;
            index[6] = 3;   index[7] = 0;
            
            index[8] = 3;    index[9] = 4;
            index[10] = 2;   index[11] = 5;
            index[12] = 0;   index[13] = 7;
            index[14] = 1;   index[15] = 6;
            
            index[16] = 4;   index[17] = 5;
            index[18] = 5;   index[19] = 6;
            index[20] = 6;   index[21] = 7;
            index[22] = 7;   index[23] = 4;
            
        }
        return index;
    }
    std::vector<GLuint>& Cube::SurfaceDrawMode(){
        if(index.size() != 36){
            index.resize(36);
            index[0] = 0;   index[1] = 1;   index[2] = 2;
            index[3] = 2;   index[4] = 3;   index[5] = 0;
            
            index[6] = 3;   index[7] = 2;   index[8] = 5;
            index[9] = 5;   index[10] = 4;   index[11] = 3;
            
            index[12] = 4;   index[13] = 5;   index[14] = 6;
            index[15] = 6;   index[16] = 7;   index[17] = 4;
            
            index[18] = 7;   index[19] = 6;   index[20] = 1;
            index[21] = 1;   index[22] = 0;   index[23] = 7;
            
            index[24] = 7;   index[25] = 0;   index[26] = 3;
            index[27] = 3;   index[28] = 4;   index[29] = 7;
            
            index[30] = 1;   index[31] = 6;   index[32] = 5;
            index[33] = 5;   index[34] = 2;   index[35] = 1;
        }
        return index;
    }
    
    
    GLuint CreateBuffer(GLenum type, GLsizeiptr size, const GLvoid* data){
        GLuint vbo;
        glGenBuffers(1,&vbo);
        glBindBuffer(type, vbo);
        glBufferData(type, size, data, GL_STATIC_DRAW);
        glBindBuffer(type, 0);
        return vbo;
    }
    
    GLuint CreateVAO(GLuint vbo, GLuint ibo){
        GLuint vao = 0;
        glGenVertexArrays(1,&vao);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,sizeof(Vertex::position)/sizeof(float),GL_FLOAT,GL_FALSE,
                              sizeof(Vertex),reinterpret_cast<GLvoid*>(offsetof(Vertex, position)));
        
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,sizeof(Vertex::color)/sizeof(float),GL_FLOAT,GL_FALSE,
                              sizeof(Vertex),reinterpret_cast<GLvoid*>(offsetof(Vertex, color)));
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBindVertexArray(0);
        return vao;
    }
    
    GLuint CompileShader(GLenum type, const GLchar* string){
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &string, nullptr);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if(!compiled){
            GLint infoLen = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
            if(infoLen){
                std::vector<char> buf;
                buf.resize(infoLen);
                if(static_cast<int>(buf.size()) >= 0){
                    glGetShaderInfoLog(shader, infoLen, NULL, buf.data());
                    std::cerr << "ERROR : Shader compile is failed\n" << buf.data() << std::endl;
                }
            }
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }
    
    GLuint CreateShaderProgram(const GLchar* vsCode, const GLchar* fsCode){
        GLuint vs = CompileShader(GL_VERTEX_SHADER, vsCode);
        GLuint fs = CompileShader(GL_FRAGMENT_SHADER, fsCode);
        if(!vs || !fs){
            return 0;
        }
        GLuint program = glCreateProgram();
        glAttachShader(program, vs);
        glDeleteShader(vs);
        glAttachShader(program, fs);
        glDeleteShader(fs);
        glLinkProgram(program);
        GLint linkStatus = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
        if(linkStatus != GL_TRUE){
            GLint infoLen = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLen);
            if(infoLen){
                std::vector<char> buf;
                buf.resize(infoLen);
                if(static_cast<int>(buf.size()) >= infoLen){
                    glGetProgramInfoLog(program, infoLen, NULL, buf.data());
                    std::cerr << "ERROR : Shader link is failed\n" << buf.data() << std::endl;
                }
            }
            glDeleteProgram(program);
            return 0;
        }
        return program;
    }
    
    const char* vsCode =
    "#version 410\n"
    "layout(location=0) in vec3 vPosition;"
    "layout(location=1) in vec4 vColor;"
    "layout(location=0) out vec4 outColor;"
    "uniform mat4x4 matMVP;"
    "void main() {"
    "   outColor = vColor;"
    "   gl_Position = matMVP * vec4(vPosition, 1.0);"
    "}";
    
    const char* fsCode =
    "#version 410\n"
    "layout(location=0) in vec4 outColor;"
    "out vec4 fragColor;"
    "void main() {"
    "   fragColor = outColor;"
    "}";
    
    //PrimitiveDrawer
    PrimitiveDrawer::~PrimitiveDrawer(){
        for(auto itr = meshes.begin(); itr != meshes.end(); ++itr){
            delete (*itr);
        }
        if(shader){
            glDeleteShader(shader);
        }
        if(vao){
            glDeleteVertexArrays(1,&vao);
        }
        if(ibo){
            glDeleteBuffers(1,&ibo);
        }
        if(vbo){
            glDeleteBuffers(1,&vbo);
        }
    }
    PrimitiveDrawer& PrimitiveDrawer::Instance(){
        static PrimitiveDrawer instance;
        return instance;
    }
    bool PrimitiveDrawer::Init(){
        vbo = CreateBuffer(GL_ARRAY_BUFFER, 1024 * 240000, nullptr);
        ibo = CreateBuffer(GL_ELEMENT_ARRAY_BUFFER, 1024 * 720000, nullptr);
        vao = CreateVAO(vbo, ibo);
        shader = CreateShaderProgram(vsCode, fsCode);
        
        if(!vbo || !ibo || !vao || !shader ){
            return false;
        }
        
        matMVPLoc = glGetUniformLocation(shader,"matMVP");
        if(matMVPLoc < 0){
            return false;
        }
        return true;
    }
    void PrimitiveDrawer::AddMesh(PrimitiveMesh* mesh){
        GLint64 vboSize = 0;
        GLint64 iboSize = 0;
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &vboSize);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glGetBufferParameteri64v(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &iboSize);
        std::vector<Vertex> verteces = mesh->Update();
        std::vector<GLuint> indices;
        if(mode == Mode::LineMode){
            indices = mesh->LineDrawMode();
        }
        else if(mode == Mode::PolygonMode){
            indices = mesh->SurfaceDrawMode();
        }
        
        GLuint iboIdx = vboEnd / sizeof(Vertex);
        for(auto& i : indices){
            i += iboIdx;
        }
        GLsizeiptr verticesBytes = sizeof(Vertex) * mesh->VertexNum();
        if(vboEnd + verticesBytes >= vboSize){
            delete mesh;
            std::cerr << "WARNING : vbo size is not enough" << std::endl;
            return;
        }
        GLsizeiptr indicesBytes = indices.size() * sizeof(GLuint);
        if(iboEnd + indicesBytes >= iboSize){
            delete mesh;
            std::cerr << "WARNING : ibo size is not enough" << std::endl;
            return;
        }
        glBufferSubData(GL_ARRAY_BUFFER, vboEnd, verticesBytes, verteces.data());
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, iboEnd, indicesBytes, indices.data());
        
        mesh->vboOffset = vboEnd;
        mesh->iboOffset = iboEnd;
        vboEnd += verticesBytes;
        iboEnd += indicesBytes;
        
        meshes.push_back(mesh);
    }
    
    void PrimitiveDrawer::Update(PrimitiveMesh* mesh){
        std::vector<Vertex> verteces = mesh->Update();
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, mesh->vboOffset, mesh->VertexNum() * sizeof(Vertex), verteces.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    void PrimitiveDrawer::Draw(const Matrix4x4& matMVP){
        glBindVertexArray(vao);
        glUseProgram(shader);
        glUniformMatrix4fv(matMVPLoc,1, GL_FALSE, &matMVP[0][0]);
        switch (mode) {
            case Mode::LineMode:
                glDrawElements(GL_LINES, iboEnd / sizeof(GLuint), GL_UNSIGNED_INT, 0);
                break;
            case Mode::PolygonMode:
                glDrawElements(GL_TRIANGLES, iboEnd / sizeof(GLuint), GL_UNSIGNED_INT, 0);
                break;
            default:
                break;
        }
        glBindVertexArray(0);
    }
    
    void PrimitiveDrawer::LineMode(){
        mode = Mode::LineMode;
        // TODO: iboの書き換え
        iboEnd = 0;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        std::vector<GLuint> indices;
        for(auto& mesh : meshes){
            iboEnd = mesh->vboOffset / sizeof(Vertex);
            std::vector<GLuint> index = mesh->LineDrawMode();
            for(int i = 0; i < index.size(); ++i){
                indices.push_back(index[i] + iboEnd);
            }
            mesh->iboOffset = iboEnd;
        }
        
        iboEnd = (GLuint)indices.size() * sizeof(GLuint);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, iboEnd, indices.data());
        
        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    }
    
    void PrimitiveDrawer::PolygonMode(){
        mode = Mode::PolygonMode;
        // TODO: iboの書き換え
        iboEnd = 0;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        std::vector<GLuint> indices;
        for(auto& mesh : meshes){
            iboEnd = mesh->vboOffset / sizeof(Vertex);
            std::vector<GLuint> index = mesh->SurfaceDrawMode();
            for(int i = 0; i < index.size(); ++i){
                indices.push_back(index[i] + iboEnd);
            }
            mesh->iboOffset = iboEnd;
        }
        
        iboEnd = (GLuint)indices.size() * sizeof(GLuint);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, iboEnd, indices.data());
        
        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
        
        //        iboEnd = 0;
        //        GLuint vboIdx = 0;
        //        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        //        GLuint* index = static_cast<GLuint*>(glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY));
        //        for(auto& mesh : meshes){
        //            vboIdx = mesh->vboOffset / sizeof(Vertex);
        //            std::vector<GLuint> indices = mesh->SurfaceDrawMode();
        //            for(int i = 0; i < indices.size(); ++i){
        //                index[i + iboEnd] = indices[i] + vboIdx;
        //            }
        //            mesh->iboOffset = iboEnd * sizeof(GLuint);
        //            iboEnd += indices.size();
        //        }
        //        iboEnd *= sizeof(GLuint);
        //        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
        //        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
    }
}

