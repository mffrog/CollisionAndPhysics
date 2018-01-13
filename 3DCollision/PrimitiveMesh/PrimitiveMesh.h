//
//  PrimitiveMesh.h
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/12.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#ifndef PrimitiveMesh_h
#define PrimitiveMesh_h

#include "Vector.h"
#include "Matrix.h"
#include "Primitive.h"
#include <vector>
#include <GL/glew.h>


namespace myTools {
    
    struct Vertex{
        Vector3 position;
        Vector4 color;
    };
    
    class PrimitiveDrawer;
    
    class PrimitiveMesh{
        friend PrimitiveDrawer;
    public:
        virtual ~PrimitiveMesh() = default;
        virtual std::vector<Vertex> Update();
        
        virtual unsigned int VertexNum() = 0;
        
        void SetColor(const Vector4& color){
            this->color = color;
        }
    protected:
        virtual std::vector<GLuint>& LineDrawMode() = 0;
        virtual std::vector<GLuint>& SurfaceDrawMode() = 0;
        
        std::vector<Vector3> vert;
        Vector4 color;

    private:
        GLuint vboOffset;
        GLuint iboOffset;
    };
    
    class LineMesh : public PrimitiveMesh {
    public:
        LineMesh();
        LineMesh(const Vector3& p1, const Vector3& p2);
        void SetPoint(const Vector3& p1, const Vector3& p2);
        void SetPoint(unsigned int, const Vector3& p);
        
        unsigned int VertexNum() override {
            return vertexNum;
        }
    private:
        static std::vector<GLuint> index;
        static unsigned int vertexNum;

        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
    };
    
    class Triangle : public PrimitiveMesh {
    public:
        Triangle();
        Triangle(const Vector3& p1, const Vector3& p2, const Vector3& p3);
        void SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3);
        void SetPoint(unsigned int, const Vector3& p);

        unsigned int VertexNum() override {
            return vertexNum;
        }
    private:
        static std::vector<GLuint> index;
        static unsigned int vertexNum;

        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
    };
    
    class Square : public PrimitiveMesh {
    public:
        Square();
        Square(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4);
        void SetPoint(const Vector3& p1, const Vector3& p2, const Vector3& p3, const Vector3& p4);
        void SetPoint(unsigned int, const Vector3& p);
        
        unsigned int VertexNum() override {
            return vertexNum;
        }
    private:
        static std::vector<GLuint> index;
        static unsigned int vertexNum;

        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
    };
    
    class Circle : public PrimitiveMesh {
    public:
        void SetRadius(const float& radius){
            this->radius = radius;
        }
        void SetDivideNum(const GLuint& divideNum){
            this->divideNum = divideNum;
        }
        
        std::vector<Vertex> Update() override;


    private:
        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
        
        float radius;
        Vector3 normal;
        int divideNum = 8;
    };
    
    class Sphere : public PrimitiveMesh {
    public:
        Sphere(unsigned int dividNum);
        void SetRadius(float radius);
        
        void SetPosition(const Vector3& position){
            this->position = position;
        }
        std::vector<Vertex> Update() override;
        unsigned int VertexNum() override {
            return (divideNum * 4) * (divideNum * 2 - 1) + 2;
        }
    private:
        void SetDivideNum(int num);
        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
        
        std::vector<GLuint> index;
        float radius = 1.0f;
        unsigned int divideNum;
        
        int topIndex = 0;
        int bottomIndex = 0;
        
        Vector3 position;
    };
    
    class CapsuleMesh : public PrimitiveMesh {
    public:
        CapsuleMesh(unsigned int divideNum);
        void SetPosition(const Vector3& pos);
        void SetLength(const Vector3& length);
        void SetRadius(const float& radius);
        std::vector<Vertex> Update() override;
        unsigned int VertexNum() override ;
    private:
        void SetDivideNum(unsigned int num);
        unsigned int divideNum;
        unsigned int sphereHalfIndex;
        int topIndex = 0;
        int bottomIndex = 0;
        float radius = 1.0f;
        Segment segment;
        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
        
        std::vector<GLuint> index;
    };
    
    class Cube : public PrimitiveMesh {
    public:
        Cube();
        void SetScale(const Vector3& s){
            scale = s;
        }
        void SetPosition(const Vector3& pos){
            position = pos;
        }
        unsigned int VertexNum() override {
            return vertexNum;
        }
        
        std::vector<Vertex> Update() override;

    private:
        static std::vector<GLuint> index;
        static unsigned int vertexNum;
        
        std::vector<GLuint>& LineDrawMode() override;
        std::vector<GLuint>& SurfaceDrawMode() override;
        
        Vector3 scale;
        Vector3 position;
    };
    
    class PrimitiveDrawer{
    public:
        enum class Mode{
            LineMode,
            PolygonMode,
        };
        
        static PrimitiveDrawer& Instance();
        bool Init();
        void AddMesh(PrimitiveMesh* mesh);
        void Update(PrimitiveMesh* mesh);
        void Draw(const Matrix4x4& matMVP);
        
        void LineMode();
        void PolygonMode();
        
    private:
        PrimitiveDrawer() = default;
        ~PrimitiveDrawer();
        PrimitiveDrawer(const PrimitiveDrawer&) = delete;
        PrimitiveDrawer& operator=(const PrimitiveDrawer) = delete;
        
        std::vector<PrimitiveMesh*> meshes;
        
        Mode mode = Mode::LineMode;
        
        GLuint vboEnd = 0;
        GLuint iboEnd = 0;
        
        GLuint vbo = 0;
        GLuint ibo = 0;
        GLuint vao = 0;
        GLuint shader = 0;
        
        GLint matMVPLoc = -1;
    };
}


#endif /* PrimitiveMesh_h */
