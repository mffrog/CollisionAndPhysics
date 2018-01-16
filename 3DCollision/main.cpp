//
//  main.cpp
//  3DCollision
//
//  Created by Tomoya Fujii on 2017/12/07.
//  Copyright © 2017年 TomoyaFujii. All rights reserved.
//

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <functional>
#include "Collision.h"
#include "PrimitiveMesh.h"
#include "Transform.h"
#include "Physics.h"
#include "Camera.h"

#define Y_ZEORO_VECTOR3(v) Vector3(v.x,0,v.z)
#define KEY_FLAG(key)   flags[Key::key]

using namespace myTools;

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
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),
                          reinterpret_cast<GLvoid*>(offsetof(Vertex,position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,sizeof(Vertex),
                          reinterpret_cast<GLvoid*>(offsetof(Vertex, color)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBindVertexArray(0);
    glDeleteBuffers(1,&vbo);
    glDeleteBuffers(1,&ibo);
    
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


struct KeyCallback{
    static std::function<void(int,int)> keyfunc;
    static void Callback(GLFWwindow* window, int key, int scancode, int action, int mods){
        if(action == GLFW_PRESS || action == GLFW_RELEASE){
            if(keyfunc){
                keyfunc(key,action);
            }
        }
    }
};
std::function<void(int,int)> KeyCallback::keyfunc = nullptr;

struct MouseClickCallback{
    static std::function<void(double,double)> func;
    static void Callback(GLFWwindow* window, int button, int action, int mods){
        if(action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT){
            if(func){
                double x,y;
                glfwGetCursorPos(window, &x, &y);
                func(x,y);
            }
        }
    }
};
std::function<void(double,double)> MouseClickCallback::func = nullptr;

struct MouseMoveCallback{
    static std::function<void(double,double)> func;
    static void Callback(GLFWwindow* window, double x, double y){
        if(func){
            func(x,y);
        }
    }
};
std::function<void(double,double)> MouseMoveCallback::func = nullptr;

template<typename Ty>
class Splitter {
public:
    void AddItem(Ty item){
        items.push_back(item);
    }
    std::vector<Ty>& GetItems(){
        return items;
    }
    int ItemNum(){
        return items.size();
    }
    
    Ty& operator[](int idx){
        if(idx >= 0 && idx < items.size()){
            return items[idx];
        }
        else {
            Ty dummy;
            return dummy;
        }
    }
    
    void Clear(){
        items.clear();
    }
private:
    std::vector<Ty> items;
};


template<typename Ty1, typename Ty2>
struct HitPair{
    MoveCollData<Ty1>& lhs;
    MoveCollData<Ty2>& rhs;
    HitPair(MoveCollData<Ty1>& lhs, MoveCollData<Ty2>& rhs) : lhs(lhs), rhs(rhs){};
    void CulsFix(float delta){
        CulcFix(delta, lhs, rhs);
    }
};


int main(int argc, const char * argv[]) {
    
    double windowX = 800.0;
    double windowY = 600.0;
    
    if(glfwInit() != GL_TRUE){
        return 1;
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    GLFWwindow* window = glfwCreateWindow(windowX, windowY, "title", nullptr, nullptr);
    if(!window){
        glfwTerminate();
        return 1;
    }
    
    glfwMakeContextCurrent(window);
    
    if(glewInit() != GLEW_OK){
        glfwTerminate();
        return 1;
    }
    
    Vector3 v[4] = {
        {0.5,0.5,0},
        {-0.5,0.5,0},
        {-0.5,-0.5,0},
        {0.5,-0.5,0},
    };
    
    Point p = {-2,2,0};
    
    PlaneCollision l;
    l.p = {0,0,0};
    l.normal = {-1,1,0};
    
    Line line;
    line.p = {1,0.5,0};
    line.v = {1,0.5,-3};
    
    PolygonCollision poly;
    poly.p[0] = {1,1,1};
    poly.p[1] = {1,-1,0};
    poly.p[2] = {1,1,-1};
    
    Segment seg;
    seg.p = {-1,0,0};
    seg.v = {3,0,0};
    
    std::cout << "\n\n\n\n";
    
    auto result = CollisionReturnFlag(seg, poly);
    
    std::cout << "result is " << (result ? "true" : "false") << std::endl;
    
    std::cout << "\n\n\n\n";

    PrimitiveDrawer& drawer = PrimitiveDrawer::Instance();
    if(!drawer.Init()){
        glfwTerminate();
        return 1;
    }

    
//    Vector3 squarePos(-5.0f,0.0f,0.0f);
//    SquareCollision squareColl;
//    squareColl.SetPoint(v[0] + squarePos, v[1] + squarePos, v[2] + squarePos, v[3] + squarePos);
//
//    auto squareMesh = new Square(v[0] + squarePos,v[1] + squarePos,v[2] + squarePos,v[3] + squarePos);
//    squareMesh->SetColor(Vector4(1,1,1,1));
//    drawer.AddMesh(squareMesh);
//
//    Vector3 floorPos(0.0f,-5.0f,0.0f);
//
//    auto floor = new Square(Vector3(-100.0f,0,-100.0f) + floorPos,Vector3(-100.0f,0,100.0f) + floorPos,
//    Vector3(100.0f,0,100.0f) + floorPos,Vector3(100.0f,0,-100.0f) + floorPos);
//    floor->SetColor({1.0f,1.0f,1.0f,1.0f});
//    drawer.AddMesh(floor);
//
//    PlaneCollision planeColl;
//    planeColl.p = floorPos;
//    planeColl.normal = {0,1,0};
    
    
//    Vector3 polyPos(-8.0f,0.0f,0.0f);
//    auto triangleMesh = new Triangle(v[0] + polyPos,v[1] + polyPos,v[2] + polyPos);
//    triangleMesh->SetColor(Vector4(1,1,1,1));
//    drawer.AddMesh(triangleMesh);
//
//    PolygonCollision polyColl(v[0] + polyPos,v[1] + polyPos,v[2] + polyPos);
    
//    auto lineMesh = new myTools::Line(v[0], v[1]);
//    lineMesh->SetColor(Vector4(0,0,1,1));
//    drawer.AddMesh(lineMesh);
    
//    auto sphere1 = new Sphere(2);
//    sphere1->SetColor({1.0f,1.0f,0.2f,1.0f});
//    sphere1->SetPosition(Vector3(-5.0f,0.0f,0.0f));
//    drawer.AddMesh(sphere1);
//    drawer.Update(sphere1);
//    auto sphere2 = new Sphere(4);
//    sphere2->SetColor({1.0f,1.0f,0.2f,1.0f});
//    sphere2->SetPosition(Vector3(-2.5f,0.0f,0.0f));
//
//    drawer.AddMesh(sphere2);
//    drawer.Update(sphere2);
//
//    Vector3 spherePos(-2.5f,0.0f,0.0f);
//    auto sphere3 = new Sphere(8);
//    sphere3->SetColor({1.0f,1.0f,1.0f,1.0f});
//    sphere3->SetPosition(spherePos);
//
//    drawer.AddMesh(sphere3);
//    drawer.Update(sphere3);
//
//    SphereCollision sphereColl;
//    sphereColl.position = spherePos;
//    sphereColl.radius = 1.0f;
    
//    auto sphere4 = new Sphere(16);
//    sphere4->SetColor({1.0f,1.0f,0.2f,1.0f});
//    sphere4->SetPosition(Vector3(2.5f,0.0f,0.0f));
//
//    drawer.AddMesh(sphere4);
//    drawer.Update(sphere4);
//
//    auto sphere5 = new Sphere(32);
//    sphere5->SetColor({1.0f,1.0f,0.2f,1.0f});
//    sphere5->SetPosition(Vector3(5.0f,0.0f,0.0f));
//
//    drawer.AddMesh(sphere5);
//    drawer.Update(sphere5);

//    Vector3 capsulePos(5.0f,-2.5f,0.0f);
//    CapsuleCollision cap(1.0f, capsulePos, Vector3(0.0f,5.0f,0));
//
//    auto capsule = new CapsuleMesh(4);
//    capsule->SetRadius(1.0f);
//    capsule->SetPosition(capsulePos);
//    capsule->SetLength(Vector3(0.0f,5.0f,0));
//    drawer.AddMesh(capsule);
//    drawer.Update(capsule);
//
//
//    Vector3 cubePos(2.5f,0.0f,0.0f);
//    auto cube = new Cube();
//    cube->SetPosition(cubePos);
//    drawer.AddMesh(cube);
//
//    CubeAABBCollision aabb;
//    aabb.max = Vector3(1.0f,1.0f,1.0f) + cubePos;
//    aabb.min = Vector3(-1.0f,-1.0f,-1.0f) + cubePos;
    
    MoveCollData<SphereCollision> moveObjData;
    moveObjData.phys.SetPosition(Vector3(-0.5,0,0),false);
    moveObjData.phys.SetMass(3.0f);
    moveObjData.collision.radius = 3.0f;
    moveObjData.collision.position = moveObjData.phys.GetPosition();
    
//    auto moveObj = new Sphere(6);
//    float ra = 3.0f;
//    moveObj->SetRadius(ra);
//    Vector3 length(0,3,0);
//    moveObj->SetPosition(moveObjData.phys.GetPosition());
//    drawer.AddMesh(moveObj);
    
    MoveCollData<SphereCollision> sphereData;
    sphereData.phys.SetPosition(Vector3(0.5f,0,0),false);
    sphereData.phys.SetMass(5.0f);
    sphereData.collision.radius = 5.0f;
    sphereData.collision.position = sphereData.phys.GetPosition();
//    auto sphere = new Sphere(8);
//    sphere->SetRadius(5.0f);
//    drawer.AddMesh(sphere);
    //SphereCollision sphereColl(1.0f,sphereData.phys.GetPosition());
    int sranT = time(NULL);
    srand(sranT);
    //srand(700);
    
    Sphere* buf;
    std::vector<Sphere*> spheres;
    std::vector<MoveCollData<SphereCollision>> sphereDatas;
    
//    spheres.push_back(sphere);
//    spheres.push_back(moveObj);
//
//    sphereDatas.push_back(sphereData);
//    sphereDatas.push_back(moveObjData);
    
    MoveCollData<CapsuleCollision> capsuleData;
    std::vector<CapsuleMesh*> caps;
    std::vector<MoveCollData<CapsuleCollision>> capDatas;
    CapsuleMesh* cBuf;
    Vector3 len(0,7,0);
    auto random = []{
        return (rand() % 255) / 255.0f;
    };
    
    auto pmRandom = []{
        return ((rand() % 255) / 255.0f) * 2.0f - 1.0f;
    };
    
    Vector3 sPos;
    
    static float threshold = 50.0f;
    static float size = 2.5f;
    
    float radius;
    
    bool useZ = true;
    
    DomeCollision dome;
    dome.minRadius = threshold * 0.8;
    dome.maxRadius = threshold + 80;
    
    Vector4 hitColor(1.0f,1.0f,0.0f,1.0f);
    Vector4 defaultColor(1.0f,1.0f,1.0f,1.0f);
    
    cBuf = new CapsuleMesh(6);
    radius = size ;//* (random() + 1);
    cBuf->SetRadius(radius);
    //len = Vector3(pmRandom(),pmRandom(),useZ ? pmRandom() : 0.0f) * 5.0f;
    cBuf->SetLength(len);
    cBuf->SetColor({random(),random(),random(),1.0f});
    caps.push_back(cBuf);
    drawer.AddMesh(cBuf);
    sPos = Vector3(pmRandom() * threshold, pmRandom() * threshold, useZ ? pmRandom() * threshold : 0.0f);
    capsuleData.phys.SetPosition(sPos,false);
    capsuleData.phys.SetVelocity(Vector3(pmRandom(), 0.0f, useZ ? pmRandom() : 0.0f) * 15.0f);
    capsuleData.phys.SetMass(radius * 5.0f);
    capsuleData.collision.s.p = sPos;
    capsuleData.collision.s.v = len;
    capsuleData.collision.radius = radius;
    capDatas.push_back(capsuleData);

    cBuf = new CapsuleMesh(6);
    radius = size ;//* (random() + 1);
    cBuf->SetRadius(radius);
    //len = Vector3(pmRandom(),pmRandom(),useZ ? pmRandom() : 0.0f) * 5.0f;
    cBuf->SetLength(len);
    cBuf->SetColor({random(),random(),random(),1.0f});
    caps.push_back(cBuf);
    drawer.AddMesh(cBuf);
    //sPos = Vector3(pmRandom() * threshold, pmRandom() * threshold, useZ ? pmRandom() * threshold : 0.0f);
    sPos = Vector3(0.0f,0.0f,0.0f);
    capsuleData.phys.SetPosition(sPos,false);
    capsuleData.phys.SetVelocity(Vector3(pmRandom(), 0.0f, useZ ? pmRandom() : 0.0f) * 10.0f);
    capsuleData.phys.SetMass(radius * 10.0f);
    capsuleData.collision.s.p = sPos;
    capsuleData.collision.s.v = len;
    capsuleData.collision.radius = radius;
    capDatas.push_back(capsuleData);
    
    int num = 15;
    
    std::vector<Cube*> cubeMeshes;
    std::vector<AABBCollision> cubeCollisions;
    Cube* cubePointer;
    AABBCollision cubeCollBuf;
    for(int i = 0; i < num; ++i){
//        cBuf = new CapsuleMesh(6);
//        radius = size ;//* (random() + 1);
//        cBuf->SetRadius(radius);
//        len = Vector3(pmRandom(),pmRandom(),useZ ? pmRandom() : 0.0f) * 5.0f;
//        cBuf->SetLength(len);
//        cBuf->SetColor({random(),random(),random(),1.0f});
//        caps.push_back(cBuf);
//        drawer.AddMesh(cBuf);
//        sPos = Vector3(pmRandom() * threshold, pmRandom() * threshold, useZ ? pmRandom() * threshold : 0.0f);
//        capsuleData.phys.SetPosition(sPos,false);
//        capsuleData.phys.SetVelocity(Vector3(pmRandom(), pmRandom(), useZ ? pmRandom() : 0.0f) * 10.0f);
//        capsuleData.phys.SetMass(radius * 5.0f);
//        capsuleData.collision.s.p = sPos;
//        capsuleData.collision.s.v = len;
//        capsuleData.collision.radius = radius;
//        capDatas.push_back(capsuleData);
        
        cubePointer = new Cube();
        sPos = Vector3(pmRandom() * threshold, pmRandom() * threshold, useZ ? pmRandom() * threshold : 0.0f);
        //sPos *= 0.7f;
        float cubeSize = size * (random()) * 12;
        Vector3 cubeWid(random() * cubeSize,random() * cubeSize, random() * cubeSize);
        //cubeWid[rand()%3] /= cubeSize;
        cubePointer->SetScale(cubeWid);
        cubePointer->SetPosition(sPos);
        cubeMeshes.push_back(cubePointer);
        drawer.AddMesh(cubePointer);
        cubeCollBuf.max = cubeWid + sPos;
        cubeCollBuf.min = -cubeWid + sPos;
        cubeCollisions.push_back(cubeCollBuf);
        
        float rate = 0.8f;
        buf = new Sphere(8);
        radius = size ;//* (random() + 1);
        buf->SetRadius(radius);
        buf->SetColor({random(),random(),random(),1.0f});

        //buf->SetColor({1,0,1,1});
        spheres.push_back(buf);
        drawer.AddMesh(buf);
        sPos = Vector3(pmRandom() * threshold, pmRandom() * threshold, useZ ? pmRandom() * threshold: 0.0f) * rate;
        print(sPos);
        sphereData.phys.SetPosition(sPos,false);
        sphereData.phys.SetVelocity(Vector3(pmRandom(), pmRandom(), useZ ? pmRandom() : 0.0f) * 10.0f);
        sphereData.phys.SetMass(radius * 3);
        sphereData.collision.position = sPos;
        sphereData.collision.radius = radius;
        sphereDatas.push_back(sphereData);
    }
    
 
    
    Camera camera;
    camera.SetPosition(Vector3(0.0f,100.0f,100.0f));
    camera.SetOrientation(Normalize(Vector3() - camera.GetPosition()));
    camera.SetUpVector(Vector3(0.0f,1.0f,0.0f));
//    Vector3 cameraPosition = {0,0,100};
//    Vector3 cameraDirection = Normalize(Vector3() - cameraPosition);
//    Vector3 UpVector = {0,1,0};
    
    float speed = 1.0f;
    
    float rad = M_PI / 240;
    bool mode;
    
    bool endFlag = false;
    
    enum Key{
        W,A,S,D,
        E,Q,
        I,J,K,L,
        U,O,
        UP,DOWN,LEFT,RIGHT,
        ENTER,
        SPACE,
        ESC,
        
        NUM,
    };
    
    bool flags[Key::NUM];
    for(auto& f : flags){
        f = false;
    }
    
    auto keyFunc = [&](int key, int action ){
        bool result = action == GLFW_PRESS ? true : false;
        switch (key) {
            case GLFW_KEY_W:
                flags[Key::W] = result;
                break;
            case GLFW_KEY_S:
                flags[Key::S] = result;
                break;
            case GLFW_KEY_D:
                flags[Key::D] = result;
                break;
            case GLFW_KEY_A:
                flags[Key::A] = result;
                break;
                
            case GLFW_KEY_E:
                KEY_FLAG(E) = result;
                break;
            case GLFW_KEY_Q:
                KEY_FLAG(Q) = result;
                break;
                
            case GLFW_KEY_I:
                KEY_FLAG(I) = result;
                break;
            case GLFW_KEY_J:
                KEY_FLAG(J) = result;
                break;
            case GLFW_KEY_K:
                KEY_FLAG(K) = result;
                break;
            case GLFW_KEY_L:
                KEY_FLAG(L) = result;
                break;
            case GLFW_KEY_U:
                KEY_FLAG(U) = result;
                break;
            case GLFW_KEY_O:
                KEY_FLAG(O) = result;
                break;
            case GLFW_KEY_UP:
                flags[Key::UP] = result;
                break;
                
            case GLFW_KEY_DOWN:
                flags[Key::DOWN] = result;
                break;
                
            case GLFW_KEY_LEFT:
                flags[Key::LEFT] = result;
                break;
                
            case GLFW_KEY_RIGHT:
                flags[Key::RIGHT] = result;
                break;
            case GLFW_KEY_ENTER:
                KEY_FLAG(ENTER) = result;
                break;
            case GLFW_KEY_SPACE:
                KEY_FLAG(SPACE) = result;
                break;
            case GLFW_KEY_ESCAPE:
                flags[Key::ESC] = result;
                break;
            default:
                break;
        }
    };
    
    
    bool skip = false;

    
    KeyCallback::keyfunc = keyFunc;
    int dataIndex = 1;
    static bool cursorMode = true;
    bool cameraMove = true;
    bool jumpRest = false;
    auto cameraFunc = [&]{
        Vector3 cameraPos = camera.GetPosition();
        Vector3 cameraOri = camera.GetOrientation();
        float power = 50;
        float jumpPower = 180;
        if(flags[Key::W]){
            if(cameraMove){
                cameraPos += cameraOri * speed;
            }
            else{
                sphereDatas[1].phys.AddAcceleration(cameraOri * speed * power);
            }
        }
        if(flags[Key::S]){
            if(cameraMove){
                cameraPos -= cameraOri * speed;
            }
            else {
                sphereDatas[1].phys.AddAcceleration(-cameraOri * speed * power);
            }
        }
        if(flags[Key::D]){
            if(cameraMove){
            cameraPos += GetRightVector(cameraOri) * speed;
            }
            else {
                sphereDatas[1].phys.AddAcceleration(GetRightVector(cameraOri) * speed * power);
            }
        }
        if(flags[Key::A]){
            if(cameraMove){
                cameraPos -= GetRightVector(cameraOri) * speed;
            }
            else {
                sphereDatas[1].phys.AddAcceleration(-GetRightVector(cameraOri) * speed * power);
            }
        }
        
        if(KEY_FLAG(E)){
            if(cameraMove){
                cameraPos.y += speed;
            }
            else {
                sphereDatas[1].phys.AddAcceleration(Vector3(0,1,0) * speed * power);
            }

//            if(!jumpRest){
//                capDatas[1].phys.AddAcceleration(Vector3(0.0f,1.0f,0.0f) * jumpPower * power);
//                jumpRest = true;
//            }
        }
        else {
            jumpRest = false;
        }
        if(KEY_FLAG(Q)){
            if(cameraMove){
            cameraPos.y -= speed;
            }
            else {
                sphereDatas[1].phys.AddAcceleration(Vector3(0,-1,0) * speed * power);
            }
        }
        camera.SetPosition(cameraPos);
        if(KEY_FLAG(UP) && cursorMode){
            Quaternion rotate = MakeQuat(GetRightVector(cameraOri), rad);
            cameraOri = rotate.rotate(cameraOri);
            camera.SetOrientation(cameraOri);
        }
        if(KEY_FLAG(DOWN) && cursorMode){
            Quaternion rotate = MakeQuat(GetRightVector(cameraOri), -rad);
            cameraOri = rotate.rotate(cameraOri);
            camera.SetOrientation(cameraOri);
        }
        if(KEY_FLAG(LEFT) && cursorMode){
            Quaternion rotate = MakeQuat(cross(GetRightVector(cameraOri), cameraOri), rad);
            cameraOri = rotate.rotate(cameraOri);
            camera.SetOrientation(cameraOri);
        }
        if(KEY_FLAG(RIGHT) && cursorMode){
            Quaternion rotate = MakeQuat(cross(GetRightVector(cameraOri), cameraOri),-rad);
            cameraOri = rotate.rotate(cameraOri);
            camera.SetOrientation(cameraOri);
        }
        static bool def = true;
        if(KEY_FLAG(ENTER)){
            if (def) {
                mode = !mode;
                if(mode){
                    cursorMode = false;
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                    //drawer.LineMode();
                }
                else {
                    cursorMode = true;
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                }
                def = false;
            }
        }
        else {
            def = true;
        }
        static bool defence = true;

        static bool mode1 = true;
        
        if(KEY_FLAG(SPACE)){
            if(defence){
                skip = !skip;
                //++dataIndex;
//                mode1 = !mode1;
//                if(mode1){
//                    drawer.LineMode();
//                }
//                else {
//                    drawer.PolygonMode();
//                }
                defence = false;
            }
        }
        else {
            defence = true;
        }
        
        if(KEY_FLAG(ESC)){
            endFlag = true;
        }
        

        //Sphere 移動処理
        if(KEY_FLAG(I)){
//            Vector3 pos;
//            Vector3 impulsePos[4] = {
//                Vector3(threshold,0,0),
//                Vector3(-threshold,0,0),
//                Vector3(0,0,threshold),
//                Vector3(0,0,-threshold)
//            };
//            float power = 10.0f;
//            for(auto& data : sphereDatas){
//                pos = data.phys.GetPosition();
//                data.phys.AddAcceleration(impulsePos[2] * (power / (pos - impulsePos[0]).Length()));
//                data.phys.AddAcceleration(impulsePos[3] * (power / (pos - impulsePos[1]).Length()));
//                data.phys.AddAcceleration(impulsePos[1] * (power / (pos - impulsePos[2]).Length()));
//                data.phys.AddAcceleration(impulsePos[0] * (power / (pos - impulsePos[3]).Length()));
//            }
            //capDatas[1].phys.AddAcceleration(Vector3(0,0,-1) * speed * power);
            speed += 0.01f;
            //dome.position += Vector3(0,0,-1) * speed;
        }
        if(KEY_FLAG(K)){
            speed -= 0.01f;
            //dome.position += Vector3(0,0,1) * speed;
            //capDatas[1].phys.AddAcceleration(Vector3(0,0,1) * speed * power);
        }
        if(KEY_FLAG(J)){
            skip = false;
            //dome.position += Vector3(-1,0,0) * speed;
            //capDatas[1].phys.AddAcceleration(Vector3(-1,0,0) * speed * power);
        }
        if(KEY_FLAG(L)){
            //dome.position += Vector3(1,0,0) * speed;
            capDatas[1].phys.AddAcceleration(Vector3(1,0,0) * speed * power);
        }
        if(KEY_FLAG(U)){
            //dome.position += Vector3(0,-1,0) * speed;
            capDatas[1].phys.AddAcceleration(Vector3(0,-1,0) * speed * power);
        }
        if(KEY_FLAG(O)){
            //dome.position += Vector3(0,1,0) * speed;
            capDatas[1].phys.AddAcceleration(Vector3(0,1,0) * speed * power * jumpPower);
        }
    };
    
    double winX = windowX;
    double winY = windowY;
    float fovy = M_PI_4;
    double winX_2 = winX * 0.5;
    double winY_2 = winY * 0.5;
    double cX;
    double cY;
    glfwGetCursorPos(window, &cX, &cY);
    Vector2 cursorPos(cX - winX_2,cY - winY_2);
    float radX = M_PI / 6 / 400;
    float radY = M_PI_4 * 0.5 / 300;
    
    float cameraRotateXSensitive = 1.0f;
    float cameraRotateYSensitive = 1.0f;
    
    Matrix4x4 fromWindowToWorld;

    auto toVec3 = [](const Vector4& v)->Vector3{
        return {v.x,v.y,v.z};
    };
    
    auto moveFunc = [&](double x, double y){
        x -= winX_2;
        y -= winY_2;
        double dx = (x - cursorPos.x) * cameraRotateXSensitive;
        double dy = (y - cursorPos.y) * cameraRotateYSensitive;;
        cursorPos.x = x;
        cursorPos.y = y;
        Vector3 cameraOri = camera.GetOrientation();
        if(cursorMode){
            cameraOri = MakeQuat(GetRightVector(cameraOri),-radY * dy ).rotate(cameraOri);
            cameraOri = MakeQuat({0,1,0}, -radX * dx).rotate(cameraOri);
            camera.SetOrientation(cameraOri);
        }
    };
    
    
    
    MouseMoveCallback::func = moveFunc;
    glfwSetCursorPosCallback(window, MouseMoveCallback::Callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glEnable(GL_DEPTH_TEST);
    
    
    //ウィンドウ幅、画角、カメラクラス
    
    auto mouseFunc = [&](double x, double y){
        std::cout << "click pos " << x << " : " << y << std::endl;
        
        //x y 座標を-1.0f ~ 1.0f に正規化
        x /= winX;
        y = 1 - (y / winY);
        x = (x * 2) - 1;
        y = (y * 2) - 1;
        
        x = 0;
        y = 0;
        Segment coll;
        float tan = tanf(fovy * 0.5f);
        Vector3 pos(tan * (winX / winY) * x,tan * y,-1);
        Vector4 start(0,0,0);
        Vector4 end = pos;
        Matrix4x4 cameraMat = Inverse(camera.GetViewMat());
        start = cameraMat * start;
        end = cameraMat * end;
        coll.p = {start.x, start.y, start.z};
        coll.v = toVec3(end - start) * 1000 ;
        for(int i = 0; i < spheres.size(); ++i){
            if(CollisionReturnFlag(sphereDatas[i].collision, coll)){
                spheres[i]->SetColor(hitColor);
            }
            else {
                //spheres[i]->SetColor(defaultColor);
            }
        }
        
//        auto ray = new LineMesh(toVec3(start),toVec3(end - start) * 10 + toVec3(start));
//        drawer.AddMesh(ray);
        
//        if(CollisionReturnFlag(coll, polygonColl)){
//            triangleMesh->SetColor({1,1,1,1});
//        }
//        else {
//            triangleMesh->SetColor({0,1,0,1});
//        }
        
//        if(CollisionReturnFlag(coll, planeColl)){
//            floor->SetColor({1,1,1,1});
//            drawer.Update(floor);
//        }
//        else {
//            floor->SetColor({1.0,0.55,0.0f,1});
//            drawer.Update(floor);
//        }
        
//        bool capAndSphere = CollisionReturnFlag(sphereColl, cap);
//        if(capAndSphere){
//            capsule->SetColor({1.0f,1.0f,0.0f,1.0f});
//            drawer.Update(capsule);
//        }
//        else {
//            capsule->SetColor({1.0f,1.0f,1.0f,1.0f});
//            drawer.Update(capsule);
//        }
//        std::cout << "capsule and sphere collison result is " << (capAndSphere ? "true" : "false") << std::endl;
//
        
//        if(CollisionReturnFlag(squareColl, coll)){
//            squareMesh->SetColor({1.0f,1.0f,0.2f,1.0f});
//            drawer.Update(squareMesh);
//        }
//        else {
//            squareMesh->SetColor({1.0f,1.0f,1.0f,1.0f});
//            drawer.Update(squareMesh);
//        }
        
        
    };
    
    MouseClickCallback::func = mouseFunc;
    
    glfwSetMouseButtonCallback(window, MouseClickCallback::Callback);
    
    glfwSetKeyCallback(window, KeyCallback::Callback);
    
    const char* vsCode =
    "#version 410\n"
    "layout(location=0) in vec3 vPosition;"
    "layout(location=1) in vec4 vColor;"
    "layout(location=0) out vec4 outColor;"
    "void main() {"
    "   outColor = vColor;"
    "   gl_Position = vec4(vPosition, 1.0);"
    "}";
    
    const char* fsCode =
    "#version 410\n"
    "layout(location=0) in vec4 outColor;"
    "out vec4 fragColor;"
    "void main() {"
    "   fragColor = outColor;"
    "}";
    
    
    //サイト
    Vertex site[4];
    site[0].position ={0.1 * 3 / 4,0,0};
    site[1].position = {-0.1 * 3 / 4,0,0};
    site[2].position = {0,0.1,0};
    site[3].position = {0,-0.1,0};
    for(auto & s : site){
        s.color = {1,1,1,1};
    }
    
    GLuint siteIndex[] = {
        0,1,2,3,
    };
    
    GLuint siteVBO = CreateBuffer(GL_ARRAY_BUFFER, sizeof(site), site);
    GLuint siteIBO = CreateBuffer(GL_ELEMENT_ARRAY_BUFFER, sizeof(siteIndex), siteIndex);
    GLuint siteVAO = CreateVAO(siteVBO, siteIBO);
    
    GLuint shader = CreateShaderProgram(vsCode, fsCode);
    if(!siteVBO || !siteIBO || !siteVAO || !shader){
        glfwTerminate();
        return 1;
    }
    
    std::vector<Splitter< MoveCollData<SphereCollision>* >> splitter(9);
    
    std::vector<HitPair<SphereCollision, SphereCollision>> spherePairs;

    Cube* cubes[6];

    AABBCollision walls[6] ;
    Vector3 cubePos(threshold,0.0f,0.0f);
    walls[0].max = Vector3(1,threshold,threshold) + cubePos;
    walls[0].min = Vector3(-1,-threshold,-threshold) + cubePos;
    cubes[0] = new Cube();
    cubes[0]->SetScale(Vector3(1,threshold, threshold));
    cubes[0]->SetPosition(cubePos);
    drawer.AddMesh(cubes[0]);

    cubePos.x = -threshold;
    walls[1].max = Vector3(1,threshold,threshold) + cubePos;
    walls[1].min = Vector3(-1,-threshold,-threshold) + cubePos;
    cubes[1] = new Cube();
    cubes[1]->SetScale(Vector3(1,threshold,threshold));
    cubes[1]->SetPosition(cubePos);
    drawer.AddMesh(cubes[1]);
    
    cubePos = Vector3(0.0f,threshold,0.0f);
    walls[2].max = Vector3(threshold,1,threshold) + cubePos;
    walls[2].min = Vector3(-threshold,-1,-threshold) + cubePos;
    cubes[2] = new Cube();
    cubes[2]->SetScale(Vector3(threshold,1,threshold));
    cubes[2]->SetPosition(cubePos);
    drawer.AddMesh(cubes[2]);
    
    cubePos *= -1;
    walls[3].max = Vector3(threshold,1,threshold) + cubePos;
    walls[3].min = Vector3(-threshold,-1,-threshold) + cubePos;
    cubes[3] = new Cube();
    cubes[3]->SetScale(Vector3(threshold,1,threshold));
    cubes[3]->SetPosition(cubePos);
    drawer.AddMesh(cubes[3]);
    
    cubePos = Vector3(0.0f,0.0f,threshold);
    walls[4].max = Vector3(threshold,threshold,1) + cubePos;
    walls[4].min = Vector3(-threshold,-threshold,-1) + cubePos;
    cubes[4] = new Cube();
    cubes[4]->SetScale(Vector3(threshold,threshold,1));
    cubes[4]->SetPosition(cubePos);
    drawer.AddMesh(cubes[4]);
    
    cubePos *= -1;
    walls[5].max = Vector3(threshold,threshold,1) + cubePos;
    walls[5].min = Vector3(-threshold,-threshold,-1) + cubePos;
    cubes[5] = new Cube();
    cubes[5]->SetScale(Vector3(threshold,threshold,1));
    cubes[5]->SetPosition(cubePos);
    drawer.AddMesh(cubes[5]);
    
//    walls[6].max = Vector3( threshold * 0.5f, 1, threshold) + cubePos;
//    walls[6].min = Vector3(-threshold * 0.5f,-1,-threshold) + cubePos;
//    walls[6].posision = Vector3(threshold * 0.5f, 0.0f, 0.0f);
    
//    walls[7].max = Vector3(threshold,1,threshold) + cubePos;
//    walls[7].min = Vector3(-threshold,-1,-threshold) + cubePos;
//    walls[7].posision = Vector3(0,threshold * 0.5,0.0);
    

//    cubes[6] = new Cube();
//    cubes[6]->SetScale(Vector3(threshold * 0.5f, 1.0f, threshold));
//    cubes[6]->SetPosition(walls[6].posision);
//    drawer.AddMesh(cubes[6]);

    
    while (!glfwWindowShouldClose(window) && !endFlag) {
        glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        
        
        cameraFunc();
        
        float delta = 1.0f / 60.0f;
        
        Vector3 gravity(0.0f,-9.8f * 15.0f,0.0f);
        gravity = Vector3();
        
        if(!skip){
        
        for(auto& data : sphereDatas){
            data.phys.AddAcceleration(gravity);
            data.phys.Update(delta, true);
        }
        
        for(auto& data : capDatas){
            data.phys.AddAcceleration(gravity);
            data.phys.Update(delta, true);
        }
        
        }
//        static int counter = 0;
//        static int interval = 10;
//        if(counter <= 0){
//            spherePairs.clear();
//            auto CulcAABB = [](MoveCollData<SphereCollision>& sphere){
//                const Vector3& pos = sphere.phys.GetPosition();
//                const Vector3& prePos = sphere.phys.GetPrePos();
//                float radius = sphere.collision.radius;
//                for(int i = 0; i < 3; ++i){
//                    if(pos[i] > prePos[i]){
//                        sphere.aabb.max[i] = pos[i] + radius;
//                        sphere.aabb.min[i] = prePos[i] - radius;
//                    }
//                    else {
//                        sphere.aabb.max[i] = prePos[i] + radius;
//                        sphere.aabb.min[i] = pos[i] - radius;
//                    }
//                }
//                sphere.aabb.posision = (pos + prePos) * 0.5f;
//            };
//            for(auto& data : sphereDatas){
//                CulcAABB(data);
//            }
//            
//            
//            int sphereDataSize = (int)sphereDatas.size();
//
//
//            for(int i = 0; i < sphereDataSize; ++i){
//                for(int j = i + 1; j < sphereDataSize; ++j){
//                    if(CollisionReturnFlag(sphereDatas[i].aabb, sphereDatas[i].aabb)){
//                        spherePairs.push_back(HitPair<SphereCollision, SphereCollision>(sphereDatas[i],sphereDatas[j]));
//                    }
//                }
//            }
//            counter = interval;
//        }
//        else {
//            --counter;
//        }
//        
//        for(auto& pair : spherePairs){
//            pair.CulsFix(delta);
//        }
        
        
        

        float maxX ;
        float minX ;
        float maxZ ;
        float minZ ;
        float maxY ;
        float minY ;
        for(int i = 0; i < sphereDatas.size(); ++i){
            Vector3 pos = sphereDatas[i].phys.GetPosition();
            Vector3 prePos = sphereDatas[i].phys.GetPrePos();

             maxX = threshold;
             minX = threshold;
             maxZ = threshold;
             minZ = threshold;
            maxY = threshold;
            minY = threshold;
            if(pos.x > prePos.x){
                maxX += pos.x + sphereDatas[i].collision.radius;
                minX += prePos.x - sphereDatas[i].collision.radius;
            }
            else {
                maxX += prePos.x + sphereDatas[i].collision.radius;
                minX += pos.x - sphereDatas[i].collision.radius;
            }
            if(pos.y > prePos.y){
                maxY += pos.y + sphereDatas[i].collision.radius;
                minY += prePos.y - sphereDatas[i].collision.radius;
            }
            else {
                maxY += prePos.y + sphereDatas[i].collision.radius;
                minY += pos.y - sphereDatas[i].collision.radius;
            }
            if(pos.z > prePos.z){
                maxZ += pos.z + sphereDatas[i].collision.radius;
                minZ += prePos.z - sphereDatas[i].collision.radius;
            }
            else {
                maxZ += prePos.z + sphereDatas[i].collision.radius;
                minZ += pos.z - sphereDatas[i].collision.radius;
            }
            float rate = 1 / threshold;
            maxX *= rate;
            minX *= rate;
            maxY *= rate;
            minY *= rate;
            maxZ *= rate;
            minZ *= rate;

            Clamp(maxX, 0.0f, 1.0f);
            Clamp(minX, 0.0f, 1.0f);
            Clamp(maxY, 0.0f, 1.0f);
            Clamp(minY, 0.0f, 1.0f);
            Clamp(maxZ, 0.0f, 1.0f);
            Clamp(minZ, 0.0f, 1.0f);

            int maxIdx = (int)maxX + (int)maxY * 2 + (int)maxZ * 4;
            int minIdx = (int)minX + (int)minY * 2 + (int)minZ * 4;
            if(maxIdx == minIdx){
                splitter[maxIdx].AddItem(&sphereDatas[i]);
            }
            else {
                splitter[8].AddItem(&sphereDatas[i]);
            }
        }

        for(auto& split : splitter){
            for(int i = 0; i < split.ItemNum(); ++i){
                for(int j = i + 1; j < split.ItemNum(); ++j){
                    CulcFix(delta, (*split[i]), (*split[j]));
                }
            }
        }
        for(int i = 0; i < splitter.size() - 1; ++i){
            for(int j = 0; j < splitter[8].ItemNum(); ++j){
                auto& rhs = splitter[i].GetItems();
                for(int k = 0; k < rhs.size(); ++k){
                    CulcFix(delta, *splitter[8][j], *rhs[k]);
                }
            }
            splitter[i].Clear();
        }
        splitter[8].Clear();
        
        
        for(int i = 0; i < sphereDatas.size(); ++i){
            for(int j = i + 1; j < sphereDatas.size(); ++j){
                CulcFix(delta, sphereDatas[i], sphereDatas[j]);
            }
        }

        for(int i = 0; i < capDatas.size(); ++i){
            for(int j = i + 1; j < capDatas.size(); ++j){
                CulcFix(delta, capDatas[i], capDatas[j]);
            }
        }

        for(int i = 0; i < sphereDatas.size(); ++i){
            for(int j = 0; j < capDatas.size(); ++j){
                CulcFix(delta, sphereDatas[i], capDatas[j]);
            }
        }
        
        for(auto& data : sphereDatas){
            data.phys.PreFix();
        }
        for(auto& data : capDatas){
            data.phys.PreFix();
        }
        

        
        
        // 当たり判定確認用
        HitData data;
        std::vector<CapsuleMesh*> hitCapMesh;
        std::vector<Sphere*> hitSphereMesh;
        
        
        for(int i = 0; i < capDatas.size(); ++i){
            for(int j = 0; j < sphereDatas.size(); ++j){
                data = MoveCollision(capDatas[i], sphereDatas[j]);
                if(data.hit){
                    hitCapMesh.push_back(caps[i]);
                    hitSphereMesh.push_back(spheres[j]);
                }
            }
        }

//        std::vector<PrimitiveMesh*> hitMeshes;
//        auto mapHitFunc = [&](float delta, auto datas){
//            HitData hitData;
//            for(int i = 0; i < datas.size(); ++i){
//                //spheres[i]->SetColor(defaultColor);
//                for(int j = 0; j < 6; ++j){
//                    //cubes[j]->SetColor(defaultColor);
//                    hitData = StaticCollision(datas[i], walls[j]);
//                    if(hitData.hit){
//                        hitMeshes.push_back(spheres[i]);
//                        hitMeshes.push_back(cubes[j]);
//                    }
//                }
//            }
//        };
//
//        mapHitFunc(delta, sphereDatas);
//        for(int i = 0; i < hitMeshes.size(); ++i){
//            hitMeshes[i]->SetColor(hitColor);
//        }
        
//        auto hitFunc = [=](auto& datas, auto& meshes, auto& hitMeshes){
//            HitData hitData;
//            for(int i = 0; i < datas.size(); ++i){
//                meshes[i]->SetColor(defaultColor);
//                for (int j = i + 1; j < datas.size(); ++j) {
//                    hitData = MoveCollision(datas[i], datas[j]);
//                    if(hitData.hit){
//                        hitMeshes.push_back(meshes[i]);
//                        hitMeshes.push_back(meshes[j]);
//                    }
//                }
//            }
//        };
        
//        hitFunc(sphereDatas,spheres,hitSphereMesh);
//        hitFunc(capDatas,caps, hitCapMesh);
        
        
//        for(auto& hit : hitCapMesh){
//            hit->SetColor(hitColor);
//        }
//
//        for(auto& hit : hitSphereMesh){
//            hit->SetColor(hitColor);
//        }
        
        
        auto mapFixFunc = [](float delta, auto& datas, auto& walls){
            for(auto& data : datas){
                for(auto& wall : walls){
                    CulcMapFix(delta, data, wall);
                }
            }
        };

        
        
//        for(auto& data : sphereDatas){
//            CulcDomeFix(delta, data, dome);
//        }
        
        static int frame = 0;
        if(!skip){
            ++frame;
        }
//        for (int i = 0; i < capDatas.size(); ++i) {
//            for(int j = 0; j < cubeCollisions.size(); ++j){
//                CulcMapFix(delta, capDatas[i], cubeCollisions[j], i,j,frame);
//            }
//        }
        
        mapFixFunc(delta,sphereDatas,cubeCollisions);
        mapFixFunc(delta,capDatas,cubeCollisions);
        mapFixFunc(delta,sphereDatas,walls);
        mapFixFunc(delta,capDatas,walls);

        if(!skip){
            for(auto& data : sphereDatas){
                data.phys.Fix();
            }
            
            for(auto& data : capDatas){
                data.phys.Fix();
            }
        }
        //mapFixFunc(delta,capDatas,cubeCollisions);
        
        auto cubeHitCheck = [&](auto& data, auto& mesh){
            for(int i = 0; i < cubeCollisions.size(); ++i){
                for(int j = 0; j < data.size(); ++j){
                    if(CollisionReturnFlag(data[j].collision, cubeCollisions[i])){
                        skip = true;
                        std::cout << "frame : " << frame << std::endl;
                        std::cout << "obj is " << j << " : aabb is " << i << std::endl;
                        std::cout << "seed is " << sranT << std::endl;
                        if(j != 1 && frame > 100){
                            skip = true;
                        }
                        cubeMeshes[i]->SetColor(hitColor);
                        mesh[j]->SetColor(hitColor);
                    }
                }
            }
        };
        

        //std::cout << "time is : ";
        //std::cout << end - start << std::endl;
        
        Vector3 pos;
        for(int i = 0; i < spheres.size(); ++i){
            pos = sphereDatas[i].phys.GetPosition();
            sphereDatas[i].collision.position = pos;
            spheres[i]->SetPosition(pos);
            drawer.Update(spheres[i]);
            spheres[i]->SetColor(defaultColor);
        }
        
        for(int i = 0; i < caps.size(); ++i){
            pos = capDatas[i].phys.GetPosition();
            capDatas[i].collision.s.p = pos;
            caps[i]->SetPosition(pos);
            drawer.Update(caps[i]);
            caps[i]->SetColor(defaultColor);
        }
        
        clock_t start = clock();
        cubeHitCheck(sphereDatas,spheres);
        cubeHitCheck(capDatas,caps);
        clock_t end = clock();
        
        for(auto& capmesh : caps){
            drawer.Update(capmesh);
        }
        
        for(int i = 0; i < 6; ++i){
            drawer.Update(cubes[i]);
        }
 
        for(int i = 0; i < cubeMeshes.size(); ++i){
            drawer.Update(cubeMeshes[i]);
            cubeMeshes[i]->SetColor(defaultColor);
        }
        
        Vector3 distance = camera.GetOrientation() * -35.0f;
        distance.y += 5.0f;
        if(!cameraMove){
            camera.SetPosition(sphereDatas[1].phys.GetPosition() + distance);
        }
        
        /*
         カメラ回転
        //cameraPosition = toVec3(RotateY(M_PI / 540) * cameraPosition);
        //cameraDirection = -cameraPosition;
        */
        const Matrix4x4 matView = camera.GetViewMat();//LookAt(camera.GetPosition(), camera + cameraPosition, UpVector);
        const Matrix4x4 matProj = Perspective(M_PI_4, windowY / windowX, 0.1, 10000.0f);
        fromWindowToWorld = Inverse(matView);
        drawer.Draw(matProj * matView);
        
        
        // サイト表示
        glBindVertexArray(siteVAO);
        glUseProgram(shader);
        glDrawElements(GL_LINES, 4, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
         
        
        glfwPollEvents();
        glfwSwapBuffers(window);
    }
    
    glDeleteShader(shader);

    glDeleteVertexArrays(1,&siteVAO);
    
    glfwTerminate();

    return 0;
}
