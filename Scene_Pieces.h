/**
Program name: Scene_Pieces.h
Purpose: place to store structs representing the objects in a scene
Programmer: Gabriel Toban Harris
Date: 2019-03-[30, 31]/2019-4-10
*/
#ifndef SCENE_PIECES_H_
#define OBJLOADER_MODIFIED_H

#include <vector>
#include <array>

#define ARRAY_SIZE 3

//There is alwasy a camera, there may be a plane or a mesh, and there may be any number of both lights and spheres.
struct Object_Light_Subproperties
{
    float diffuse_colour [ARRAY_SIZE];//"diffuse color of the light"
    float specular_colour [ARRAY_SIZE];//"specular color of the light"
};

struct Object_Light_Properties : Object_Light_Subproperties
{
    float shininess;//"specular shininess factor"
    float ambient_colour [ARRAY_SIZE];//"ambient color of the object"
};

struct Camera
{
    int field_of_view;//"where theta is the field-of-view in degrees"
    int focal_length;//"focal length of the camera"
    float aspect_ratio;//"aspect ratio of the camera"
    float position [ARRAY_SIZE];//camera position
}camera_instance;

struct Plane : Object_Light_Properties
{
    bool active = false;//boolean for if struct is in use
    float position [ARRAY_SIZE];//"position of a point on the plane"
    float normal [ARRAY_SIZE];//plane geometric normal
}plane_instance;

struct Sphere : Object_Light_Properties
{
    float radius;//radius of sphere
    float position [ARRAY_SIZE];//"position of the center of the sphere"
};

struct Mesh : Object_Light_Properties
{
    bool active = false;//boolean for if struct is in use
    char * filename;//"where filename.obj is the OBJ file containing the mesh"
    std::vector<std::array<float, ARRAY_SIZE>> vertices;//vertices defining the mesh
}mesh_instance;

struct Light : Object_Light_Subproperties
{
    float position [ARRAY_SIZE];//"position of the light"
};

#endif /* SCENE_PIECES_H_ */
