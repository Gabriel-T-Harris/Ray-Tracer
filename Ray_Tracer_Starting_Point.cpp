/**
Program name: Ray_Tracer_Starting_Point.cpp
Purpose: ray tracer driver
Programmer: Gabriel Toban Harris
Date: 2019-03-[30, 31]/2019-4-[5, 10]
*/
#ifndef RAY_TRACER_CPP
#define RAY_TRACER_CPP

#include "Scene_Pieces.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include "CImg.h"
#include <utility>
#include <float.h>
#include "OBJloader_modified.h"
#include <string.h>

//#define DEBUG_1//file reading
//#define DEBUG_2//paths and display output
//#define DEBUG_3_MISS//intersections miss, warning is very time consuming
//#define DEBUG_3_HIT//intersection hit, warning is very time consuming
//#define DEBUG_4_BLOCKED//light ray blocked by object, warning is a bit time consuming
//#define DEBUG_4_NOT_BLOCKED//light ray was not blocked by object, warning is rather time consuming
#define ZERO_TOLERANCE 0.0005f//0.0005f is magic number that acts as tolerance to see if the value is approximately == to 0.
#define SHADOW_BIAS 0.05f//0.05f is magic number for removing some shadows

using std::endl;
using std::cerr;

static float dot_product(const float [ARRAY_SIZE], const float [ARRAY_SIZE]);//forward declaration to use function in...plane_intersection(...).
static void cross_product(const float [ARRAY_SIZE], const float [ARRAY_SIZE], float [ARRAY_SIZE]);//forward declaration to use function in...triangle_intersection(...).

/**
 * function to read int from file
 *
 * Takes in file being read and returns found int.
 *
 * input_file: file being read from
 */
static int file_read_int(std::fstream& input_file)
{
    std::string placeholder;
    std::getline(input_file, placeholder, ' ');//discard first part
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    std::getline(input_file, placeholder);
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    return std::stoi(placeholder);
}

/**
 * function to read float from file
 *
 * Takes in file being read and returns found float.
 *
 * input_file: file being read from
 */
static float file_read_float(std::fstream& input_file)
{
    std::string placeholder;
    std::getline(input_file, placeholder, ' ');//discard first part
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    std::getline(input_file, placeholder);
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    return std::stof(placeholder);
}

/*
 * Calculates dot product (also known as scalar product). Takes in mathematical vectors and returns their dot product. TODO: Make a version of this that is both generic and n sized, for fun.
 *
 * VECTOR_1: first vector
 * VECTOR_2: second vector
 */
static float dot_product(const float VECTOR_1 [ARRAY_SIZE], const float VECTOR_2 [ARRAY_SIZE])
{
    float to_return = 0.0f;
    for (unsigned int i = 0; i < ARRAY_SIZE; ++i)
        to_return += VECTOR_1[i] * VECTOR_2[i];
    return to_return;
}
static float dot_product(const float VECTOR_1 [ARRAY_SIZE], const std::array<float, ARRAY_SIZE>& VECTOR_2)
{
    float to_return = 0.0f;
    for (unsigned int i = 0; i < ARRAY_SIZE; ++i)
        to_return += VECTOR_1[i] * VECTOR_2[i];
    return to_return;
}

/*
 * Calculates intersection of a plane with a ray. Returns nullptr in event of no intersection. Returns dynamically allocated float which represents a scaler to point of intersection.
 * Also returns nullptr in event that intersection us not positive for implementation reasons. Such as nullptr being used as a special value.
 *
 * INPUT_PLANE: is plane being tested for an intersection
 * RAY_ORIGIN: array of size 3 represent point origin of the ray
 * RAY_DIRECTION: array size 3 representing a mathematical vector of the ray's direction
 */
static float * plane_intersection(const struct Plane& INPUT_PLANE, const float RAY_ORIGIN [ARRAY_SIZE], const float RAY_DIRECTION [ARRAY_SIZE])
{
    const float RAY_DIRECTION_DOT_NORMAL = dot_product(RAY_DIRECTION, INPUT_PLANE.normal);

    //no intersection
    if (-ZERO_TOLERANCE < RAY_DIRECTION_DOT_NORMAL && RAY_DIRECTION_DOT_NORMAL < ZERO_TOLERANCE)
        return nullptr;//lines are parallel thus no intersection
    //intersection
    else
    {
        const float POSITION_MINUS_RAY_ORIGIN [ARRAY_SIZE] = {INPUT_PLANE.position[0] - RAY_ORIGIN[0], INPUT_PLANE.position[1] - RAY_ORIGIN[1], INPUT_PLANE.position[2] - RAY_ORIGIN[2]};

        const float PLACEHOLDER = dot_product(POSITION_MINUS_RAY_ORIGIN, INPUT_PLANE.normal) / RAY_DIRECTION_DOT_NORMAL;
        //positive value
        if (PLACEHOLDER > 0.0f)
        {
            float * to_return = new float;
            *to_return = PLACEHOLDER;
            return to_return;
        }
        //not positive
        else
            return nullptr;
    }
}

/*
 * Used to determine if a ray intersects with a sphere. Calculates Sphere intersection through quadratic equation. Returns dynamically allocated float array. Value of 1st index determines case/number of intersections.
 *
 * case [0] == 2 is as follows:
 * Returned float array is size 3. Means 2 intersections. Thus means that the both indices 1 and 2 contain useful information.
 *
 * case [0] == 1 is as follows:
 * Returned float array is size 2. Means one intersection so that sole intersection is stored in index 1
 *
 * case [0] == 0 is as follows:
 * Returned float array is size 1. 0 means no intersections with sphere.
 *
 * INPUT_SPHERE: is sphere being tested for an intersection
 * RAY_ORIGIN: array of size 3 represent point origin of the ray
 * RAY_DIRECTION: array size 3 representing a mathematical vector of the ray's direction
 *
 * Note: RAY_DIRECTION is assumed to be should be normalized, thus QUADRATIC_A would be equal to 1.
 */
static float * sphere_intersection(const struct Sphere& INPUT_SPHERE, const float RAY_ORIGIN [ARRAY_SIZE], const float RAY_DIRECTION [ARRAY_SIZE])
{
    //float[] before floats because floats are calculated from float[].
    const float QUARATIC_ORIGIN_MINUS_CENTER [ARRAY_SIZE] = {RAY_ORIGIN[0] - INPUT_SPHERE.position[0], RAY_ORIGIN[1] - INPUT_SPHERE.position[1], RAY_ORIGIN[2] - INPUT_SPHERE.position[2]};//Store repeated subtractions between RAY_ORIGIN and INPUT_SPHERE's center. Float to avoid future repeated conversions to float.
    const float QUADRATIC_B = (RAY_DIRECTION[0] * QUARATIC_ORIGIN_MINUS_CENTER[0] + RAY_DIRECTION[1] * QUARATIC_ORIGIN_MINUS_CENTER[1] +
                               RAY_DIRECTION[2] * QUARATIC_ORIGIN_MINUS_CENTER[2]) * 2.0f;
    const float DETERMINANT = pow(QUADRATIC_B, 2.0f) - (pow(QUARATIC_ORIGIN_MINUS_CENTER[0], 2.0f) + pow(QUARATIC_ORIGIN_MINUS_CENTER[1], 2.0f) +
                              pow(QUARATIC_ORIGIN_MINUS_CENTER[2], 2.0f) - pow(INPUT_SPHERE.radius, 2.0f)) * 4.0f;

    {
        float * to_return;
        //no intersectionz
        if (DETERMINANT < 0.0f)//DETERMINANT is negative
        {
            to_return = new float[1];
            to_return[0] = 0.0f;
        }
        //2 intersections
        else if (DETERMINANT > 0.0f)//DETERMINANT is positive
        {
            const float PLACEHOLDER = sqrt(DETERMINANT);
            to_return = new float[3];
            to_return[0] = 2.0f;
            to_return[1] = (-QUADRATIC_B - PLACEHOLDER) / 2.0f;
            to_return[2] = (-QUADRATIC_B + PLACEHOLDER) / 2.0f;
        }
        //1 intersection
        else //DETERMINANT == 0
        {
            to_return = new float[2];
            to_return[0] = 1.0f;
            to_return[1] = -QUADRATIC_B / 2.0f;
        }

        return to_return;
    }
}

/*
 * Used to determine if a ray intersects with a triangle, meant to be used with Mesh.
 *
 * VERTEX_1, VERTEX_2, VERTEX_3: are 3 vertices that define a triangle
 * RAY_ORIGIN: array of size 3 represent point origin of the ray
 * RAY_DIRECTION: array size 3 representing a mathematical vector of the ray's direction
 * Note: RAY_DIRECTION is assumed to be should be normalized
 */
static float * triangle_intersection(const std::array<float, ARRAY_SIZE>& VERTEX_1, const std::array<float, ARRAY_SIZE>& VERTEX_2, const std::array<float, ARRAY_SIZE>& VERTEX_3,
                                     const float RAY_ORIGIN [ARRAY_SIZE], const float RAY_DIRECTION [ARRAY_SIZE])
{
    float triangle_normal [ARRAY_SIZE];
    {
        float first_vector_2_minus_1 [ARRAY_SIZE], second_vector_3_minus_1 [ARRAY_SIZE];
        for (unsigned int i = 0; i < ARRAY_SIZE; ++i)
        {
            first_vector_2_minus_1[i] = VERTEX_2[i] - VERTEX_1[i];
            second_vector_3_minus_1[i] = VERTEX_3[i] - VERTEX_1[i];
        }
        cross_product(first_vector_2_minus_1, second_vector_3_minus_1, triangle_normal);
    }

    {
        float * to_return;
        {
            const float NORMAL_DOT_DIRECTION = dot_product(triangle_normal, RAY_DIRECTION);

            if (-ZERO_TOLERANCE < NORMAL_DOT_DIRECTION && NORMAL_DOT_DIRECTION < ZERO_TOLERANCE)
                return nullptr;//lines are parallel thus no intersection;
            {
                const float PLACEHOLDER = (dot_product(triangle_normal, RAY_ORIGIN) + dot_product(triangle_normal, VERTEX_1)) / NORMAL_DOT_DIRECTION;

                if (PLACEHOLDER < 0.0f)//not negative value check
                    return nullptr;

                to_return = new float;
                *to_return = PLACEHOLDER;
            }

        }
        {
            unsigned int i;
            float placeholder [ARRAY_SIZE], cross_product_result [ARRAY_SIZE], cross_first_vector [ARRAY_SIZE], cross_second_vector [ARRAY_SIZE];//reused for each edge test
            for (i = 0; i < ARRAY_SIZE; ++i)
                placeholder[i] = RAY_ORIGIN[i] + *to_return * RAY_DIRECTION[i];

            //test edges, one at a time to fail fast
            for (i = 0; i < ARRAY_SIZE; ++i)
            {
                cross_first_vector[i] = VERTEX_2[i] - VERTEX_1[i];
                cross_second_vector[i] = placeholder[i] - VERTEX_1[i];
            }
            cross_product(cross_first_vector, cross_second_vector, cross_product_result);
            if (dot_product(triangle_normal, cross_product_result) < 0.0f)
            {
                delete to_return;
                return nullptr;
            }

            for (i = 0; i < ARRAY_SIZE; ++i)
            {
                cross_first_vector[i] = VERTEX_3[i] - VERTEX_2[i];
                cross_second_vector[i] = placeholder[i] - VERTEX_2[i];
            }
            cross_product(cross_first_vector, cross_second_vector, cross_product_result);
            if (dot_product(triangle_normal, cross_product_result) < 0.0f)
            {
                delete to_return;
                return nullptr;
            }

            for (i = 0; i < ARRAY_SIZE; ++i)
            {
                cross_first_vector[i] = VERTEX_1[i] - VERTEX_2[i];
                cross_second_vector[i] = placeholder[i] - VERTEX_3[i];
            }
            cross_product(cross_first_vector, cross_second_vector, cross_product_result);
            if (dot_product(triangle_normal, cross_product_result) < 0.0f)
            {
                delete to_return;
                return nullptr;
            }
        }
        return to_return;
    }
}

/**
 * method to read int array 3 from file.
 *
 * Takes in the file being read and the size 3 int array to store the values.
 *
 * input_file: file being read from
 * int_container: where the 3 read ints are stored
 */
static void file_read_int_array_3(std::fstream& input_file, int int_container [ARRAY_SIZE])
{
    std::string placeholder;
    std::getline(input_file, placeholder, ' ');//discard first part
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    for (int j = 0; j < 2; ++j)
    {
        std::getline(input_file, placeholder, ' ');
        int_container[j] = std::stoi(placeholder);
        #ifdef DEBUG_1
            cerr << placeholder << endl;
        #endif
    }
    std::getline(input_file, placeholder);//get remainder
    int_container[2] = std::stoi(placeholder);
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
}

/**
 * method to read float array 3 from file.
 *
 * Takes in the file being read and the size 3 float array to store the values.
 *
 * input_file: file being read from
 * int_container: where the 3 read floats are stored
 */
static void file_read_float_array_3(std::fstream& input_file, float float_container [ARRAY_SIZE])
{
    std::string placeholder;
    std::getline(input_file, placeholder, ' ');//discard first part
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
    for (int j = 0; j < 2; ++j)
    {
        std::getline(input_file, placeholder, ' ');
        float_container[j] = std::stof(placeholder);
        #ifdef DEBUG_1
            cerr << placeholder << endl;
        #endif
    }
    std::getline(input_file, placeholder);//get remainder
    float_container[2] = std::stof(placeholder);
    #ifdef DEBUG_1
        cerr << placeholder << endl;
    #endif
}

/**
  * method to setup object_light_subproperties part.
 *
 * input_file: file being read from
 * input_struct: where the read values are stored
 */
static void file_read_setup_object_light_subproperties(std::fstream& input_file, struct Object_Light_Subproperties& input_struct)
{
    file_read_float_array_3(input_file, input_struct.diffuse_colour);//diffuse colour
    file_read_float_array_3(input_file, input_struct.specular_colour);//specular colour
}

/**
 * method to setup object_light_properties part.
 *
 * input_file: file being read from
 * input_struct: where the read values are stored
 */
static void file_read_setup_object_light_properties(std::fstream& input_file, struct Object_Light_Properties& input_struct)
{
    file_read_float_array_3(input_file, input_struct.ambient_colour);//ambient colour
    file_read_setup_object_light_subproperties(input_file, input_struct);//setup object_light_subproperties
    input_struct.shininess = file_read_float(input_file);//shininess
}

/*
 * Method for creating a normalized ray direction. ray_direction is a mathematical vector of size 3. TODO: Make a version of this that is both generic and n sized, for fun.
 *
 * ray_direction: is where the computed values are stored
 * RAY_ORIGIN: is the source of the ray
 * RAY_TARGET: well it is meant to be the target of the ray, it can be any point along the the desired line
 */
static void create_normailized_ray_direction(const float RAY_ORIGIN [ARRAY_SIZE], const int RAY_TARGET [ARRAY_SIZE], float ray_direction [ARRAY_SIZE])
{
    unsigned int i;
    //intialize ray_direction
    for (i = 0; i < ARRAY_SIZE; ++i)
        ray_direction[i] = static_cast<float>(RAY_TARGET[i] - RAY_ORIGIN[i]);
    //normalize ray_direction
    {
        const float UNNORMALIZED_RAY_DIRECTION_LENGTH = sqrt(pow(ray_direction[0], 2.0f) + pow(ray_direction[1], 2.0f) + pow(ray_direction[2], 2.0f));//calculate vector length
        for (i = 0; i < ARRAY_SIZE; ++i)
            ray_direction[i] /= UNNORMALIZED_RAY_DIRECTION_LENGTH;
    }
}
static void create_normailized_ray_direction(const float RAY_ORIGIN [ARRAY_SIZE], const float RAY_TARGET [ARRAY_SIZE], float ray_direction [ARRAY_SIZE])
{
    unsigned int i;
    //intialize ray_direction
    for (i = 0; i < ARRAY_SIZE; ++i)
        ray_direction[i] = RAY_TARGET[i] - RAY_ORIGIN[i];
    //normalize ray_direction
    {
        const float UNNORMALIZED_RAY_DIRECTION_LENGTH = sqrt(pow(ray_direction[0], 2.0f) + pow(ray_direction[1], 2.0f) + pow(ray_direction[2], 2.0f));//calculate vector length
        for (i = 0; i < ARRAY_SIZE; ++i)
            ray_direction[i] /= UNNORMALIZED_RAY_DIRECTION_LENGTH;
    }
}

/*
 * Calculates cross product. Takes in mathematical vectors and stores result in float result [3] TODO: Make a version of this that is both generic and n sized (maybe), for fun.
 *
 * VECTOR_1 and VECTOR_2: are mathematical vectors being crossed
 * result: stores the function's 'output'
 */
static void cross_product(const float VECTOR_1 [ARRAY_SIZE], const float VECTOR_2 [ARRAY_SIZE], float result [ARRAY_SIZE])
{
    result[0] = VECTOR_1[1] * VECTOR_2[2] - VECTOR_1[2] * VECTOR_2[1];
    result[1] = VECTOR_1[2] * VECTOR_2[0] - VECTOR_1[0] * VECTOR_2[2];
    result[2] = VECTOR_1[0] * VECTOR_2[1] - VECTOR_1[1] * VECTOR_2[0];
}


int main(int name_of_arguments, char * argument_container [])
{
    const std::string FILE_NAME = name_of_arguments > 1 ? argument_container[1] : /*"mesh_scene1"*/"scene1";
    std::vector<struct Sphere> sphere_container;
    std::vector<struct Light> light_container;

    //file reading, if the order of the scene object's attributes was unknown, then could use the discarded part to figure out which attribute.
    {
        const std::string INPUT_FILE_PATH =
            #ifdef ABSOLUTE_PATH
                std::string(ABSOLUTE_PATH) +
            #endif
            "Input/" + FILE_NAME + ".txt";
        std::fstream target_file = std::fstream(INPUT_FILE_PATH , std::fstream::in);

        if (target_file.is_open())
        {
            const int OBJECT_COUNT = target_file.get() - '0';
            std::string placeholder;
            target_file.get();
            #ifdef DEBUG_1
                cerr << OBJECT_COUNT << endl;
            #endif
            for (int i = 0; i < OBJECT_COUNT; ++i)
            {
                std::getline(target_file, placeholder);
                #ifdef DEBUG_1
                    cerr << placeholder << endl;
                #endif
                if (placeholder == "camera")
                {
                    file_read_float_array_3(target_file, camera_instance.position);//position
                    camera_instance.field_of_view = file_read_int(target_file);//field-of-view
                    camera_instance.focal_length = file_read_int(target_file);//focal length
                    camera_instance.aspect_ratio = file_read_float(target_file);//aspect ratio
                }
                else if (placeholder == "plane")
                {
                    plane_instance.active = true;//note that there is a plane in scene
                    file_read_float_array_3(target_file, plane_instance.normal);//normal
                    file_read_float_array_3(target_file, plane_instance.position);//position
                    file_read_setup_object_light_properties(target_file, plane_instance);//setup object_light_properties part
                }
                else if (placeholder == "sphere")
                {
                    sphere_container.push_back(Sphere());
                    file_read_float_array_3(target_file, sphere_container.back().position);//position
                    sphere_container.back().radius = file_read_float(target_file);//radius
                    file_read_setup_object_light_properties(target_file, sphere_container.back());//setup object_light_properties part
                }
                else if (placeholder == "mesh")
                {
                    mesh_instance.active = true;//note that there is a mesh in plane
                    std::getline(target_file, placeholder, ' ');//discard first part
                    #ifdef DEBUG_1
                        cerr << placeholder << endl;
                    #endif
                    {
                        std::string filename_plane_holder;
                        std::getline(target_file, filename_plane_holder);//filename
                        #ifdef DEBUG_1
                            cerr << filename_plane_holder << endl;
                        #endif
                        filename_plane_holder =
                        #ifdef ABSOLUTE_PATH
                            std::string(ABSOLUTE_PATH) +
                        #endif
                        "Input/" + filename_plane_holder;
                        mesh_instance.filename = new char[filename_plane_holder.length() + 1];
                        strcpy(mesh_instance.filename, filename_plane_holder.c_str());
                    }
                    file_read_setup_object_light_properties(target_file, mesh_instance);//setup object_light_properties part
                    loadOBJ(mesh_instance.filename, mesh_instance.vertices);
                }
                else if (placeholder == "light")
                {
                    light_container.push_back(Light());
                    file_read_float_array_3(target_file, light_container.back().position);//position
                    file_read_setup_object_light_subproperties(target_file, light_container.back());//setup object_light_subproperties part
                }
                else
                {
                    cerr << "This message should never appear. Read value is \"" << placeholder << "\"" << endl;
                }
            }
            sphere_container.shrink_to_fit();
            light_container.shrink_to_fit();
        }
        else
            cerr << "Error: unable to open \"" << INPUT_FILE_PATH << "\"" << endl;
    }

    {
        cimg_library::CImg<float> output_image;
        //ray trace
        {
            unsigned int corresponding_index;
            const float TAN_CALCULATION = tan(camera_instance.field_of_view / 2.0f * 3.14159265f / 180.0f);//3.14159265f / 180.0f to convert from degrees to radians, is used to define subsequent values
            const unsigned int IMAGE_VERTICAL = static_cast<unsigned int>(TAN_CALCULATION * camera_instance.focal_length * 2),
                               HALF_IMAGE_VERTICLE = IMAGE_VERTICAL >> 1, IMAGE_HORIZONTAL = static_cast<unsigned int>(camera_instance.aspect_ratio * IMAGE_VERTICAL),
                               HALF_IMAGE_HORIZONTAL = IMAGE_HORIZONTAL >> 1;
            int current_ray_target [ARRAY_SIZE];//variable is reused
            float smallest_distance_scalar;//Values to avoid constantly assigning placeholder.first a new value, figure the assignment will be faster this way.
            float * intersections_placeholder;
            float orginal_ray_direction [ARRAY_SIZE], intersection_point_normal [ARRAY_SIZE], intersection_point [ARRAY_SIZE] /*new origin, for new rays starting from previous ray's chosen intersection going to light sources*/;
            const float ADJUSTED_CAMERA_POSITION [ARRAY_SIZE] = {camera_instance.position[0], camera_instance.position[1], static_cast<float>(camera_instance.position[2] + camera_instance.focal_length)};
            std::pair<const struct Object_Light_Properties *, float> placeholder;//{intersected object, intersection distance from camera in terms of scalar}
            output_image = cimg_library::CImg<float>(IMAGE_HORIZONTAL, IMAGE_VERTICAL, 1, 3, 0);//{R, G, B}
            current_ray_target[2] = -1;

            //TODO: consider mulithreading the the for loop interactions
            for (unsigned int x = 0; x < IMAGE_HORIZONTAL; ++x)
            {
                current_ray_target[0] = static_cast<int>(x - HALF_IMAGE_HORIZONTAL);//subtraction is offset
                for(unsigned int y = 0; y < IMAGE_VERTICAL; ++y)
                {
                    //reset placeholder values
                    placeholder.first = nullptr;
                    placeholder.second = FLT_MAX;//Slight problem if the closest intersection point is at FLT_MAX as scalar, though is improbable.
                    current_ray_target[1] = static_cast<int>(HALF_IMAGE_VERTICLE - y);//subtraction is offset
                    create_normailized_ray_direction(ADJUSTED_CAMERA_POSITION, current_ray_target, orginal_ray_direction);

                    //original ray intersections
                    if (plane_instance.active)//a plan exists
                    {
                        intersections_placeholder = plane_intersection(plane_instance, camera_instance.position, orginal_ray_direction);

                        if (intersections_placeholder != nullptr)
                        {
                            #ifdef DEBUG_3_HIT
                                cerr << "There is an intersection with plane_instance, intersections_placeholder value " << *intersections_placeholder << endl;
                            #endif
                            if (*intersections_placeholder < placeholder.second)
                            {
                                #ifdef DEBUG_3_HIT
                                    cerr << "New closer point found with plane_instance." << endl;
                                #endif
                                placeholder.first = &plane_instance;
                                placeholder.second = *intersections_placeholder;
                                for (unsigned int i = 0; i < ARRAY_SIZE; ++i)
                                {
                                    intersection_point[i] = placeholder.second * orginal_ray_direction[i] + camera_instance.position[i];
                                    intersection_point_normal[i] = plane_instance.normal[i];
                                }
                            }

                            delete intersections_placeholder;
                        }
                        #ifdef DEBUG_3_MISS
                            else
                                cerr << "There is no intersection with plane_instance, with intersections_placeholder value " << *intersections_placeholder << endl;
                        #endif
                    }
                    if (!sphere_container.empty())//spheres exist
                    {
                        smallest_distance_scalar = FLT_MAX;

                        for (unsigned int index = 0; index < sphere_container.size(); ++index)
                        {
                            intersections_placeholder = sphere_intersection(sphere_container[index], camera_instance.position, orginal_ray_direction);

                            #ifdef DEBUG_3_HIT //&& intersections_placeholder[0] > 0
                                if (intersections_placeholder[0] > 0)
                                    cerr << "There are " << intersections_placeholder[0] << " intersections with sphere_container[" << index << "]." << endl;
                            #endif
                            #ifdef DEBUG_3_MISS //&& intersections_placeholder[0] < 1
                                if (intersections_placeholder[0] < 1)
                                    cerr << "No intersection with sphere_container[" << index << "], value of intersections_placeholder[0] is " << intersections_placeholder[0] << endl;
                            #endif
                            for (int i = 1; i < intersections_placeholder[0] + 1; ++i)//+ 1 is an offset for the indices
                            {
                                #ifdef DEBUG_3_HIT
                                    cerr << "value of intersections_placeholder[" << i << "] is " << intersections_placeholder[i] << endl;
                                #endif
                                if (-1.0f < intersections_placeholder[i] && intersections_placeholder[i] < smallest_distance_scalar)
                                {
                                    smallest_distance_scalar = intersections_placeholder[i];
                                    corresponding_index = index;
                                }
                            }

                            delete[] intersections_placeholder;//clear memory
                        }

                        if (-1.0f < smallest_distance_scalar && smallest_distance_scalar < placeholder.second)
                        {
                            #ifdef DEBUG_3_HIT
                                cerr << "New closer point found with Sphere." << endl;
                            #endif
                            placeholder.first = &sphere_container[corresponding_index];
                            placeholder.second = smallest_distance_scalar;
                            for (unsigned int i = 0; i < ARRAY_SIZE; ++i)
                                intersection_point_normal[i] = ((intersection_point[i] = placeholder.second * orginal_ray_direction[i] + camera_instance.position[i])
                                                                - sphere_container[corresponding_index].position[i]) / sphere_container[corresponding_index].radius;
                        }
                    }
                    if (mesh_instance.active)//mesh exists
                    {
                        smallest_distance_scalar = FLT_MAX;

                        for (unsigned int index = 0; index < mesh_instance.vertices.size(); index += 3)//3 vertices make a triangle
                        {
                            intersections_placeholder = triangle_intersection(mesh_instance.vertices[index], mesh_instance.vertices[index + 1], mesh_instance.vertices[index + 2],
                                                                              camera_instance.position, orginal_ray_direction);
                            if (intersections_placeholder != nullptr)
                            {
                                #ifdef DEBUG_3_HIT
                                    cerr << "Intersection with triangle formed by mesh_instance.vertices[" << index << ", " << index + 2 << "]" << endl;
                                #endif
                                if (*intersections_placeholder < smallest_distance_scalar)
                                {
                                    smallest_distance_scalar = *intersections_placeholder;
                                    corresponding_index = index;
                                }
                            }
                            #ifdef DEBUG_3_MISS
                                else
                                    cerr << "No Intersection with triangle formed by mesh_instance.vertices[" << index << ", " << index + 2 << "]" << endl;
                            #endif

                            delete intersections_placeholder;//clear memory
                        }

                        if (-1.0f < smallest_distance_scalar && smallest_distance_scalar < placeholder.second)
                        {
                            #ifdef DEBUG_3_HIT
                                cerr << "New closer point found with Mesh triangle." << endl;
                            #endif
                            placeholder.first = &mesh_instance;
                            placeholder.second = smallest_distance_scalar;
                            //calculate normal
                            {
                                unsigned int i;
                                float first_vector [ARRAY_SIZE], second_vector [ARRAY_SIZE];
                                for (i = 0; i < ARRAY_SIZE; ++i)
                                {
                                    intersection_point[i] = placeholder.second * orginal_ray_direction[i] + camera_instance.position[i];
                                    first_vector[i] = (mesh_instance.vertices[corresponding_index + 1])[i] - (mesh_instance.vertices[corresponding_index])[i];
                                    second_vector[i] = (mesh_instance.vertices[corresponding_index + 2])[i] - (mesh_instance.vertices[corresponding_index])[i];
                                }
                                cross_product(first_vector, second_vector, intersection_point_normal);
                                //normalize
                                {
                                    const float UNNORMALIZED_LENGTH = sqrt(pow(intersection_point_normal[0], 2.0f) + pow(intersection_point_normal[1], 2.0f) + pow(intersection_point_normal[2], 2.0f));//calculate vector length
                                    for (i = 0; i < ARRAY_SIZE; ++i)
                                        intersection_point_normal[i] /= UNNORMALIZED_LENGTH;
                                }
                            }
                        }
                    }

                    //calculates illumination
                    if (placeholder.first != nullptr)
                    {
                        unsigned int i;//outer for loop counter
                        {
                            bool intersection_continue;//True means intersection found, false means no intersection. Is used to skip a a given loop iteration without having to recheck for an intersection.
                            unsigned int j;//inner for loop counter
                            float scalar_to_light;//calculate scalar to current light, acts as an upper bound
                            float * placeholder_2;//is reused for various intersections
                            float light_ray_direction [ARRAY_SIZE];//note points towards light from ray origin

                            for (i = 0; i < light_container.size(); ++i)
                            {
                                //initialize loop specific values
                                create_normailized_ray_direction(intersection_point, light_container[i].position, light_ray_direction);//new direction for new light
                                for (j = 0; j < ARRAY_SIZE; ++j)
                                    if ((scalar_to_light = light_container[i].position[j] / light_ray_direction[j]) > ZERO_TOLERANCE)//To make sure a 0 value in a given direction does not screw over calculations.
                                        break;//Exit when a positive value has been found as it is a scalar thus should be the same for all the others that are not 0.

                                //light ray intersection test
                                if (plane_instance.active)//a plan exists
                                {
                                    placeholder_2 = plane_intersection(plane_instance, intersection_point, light_ray_direction);

                                    if (placeholder_2 != nullptr)//thus intersection exist
                                    {
                                        if (SHADOW_BIAS < *placeholder_2 && *placeholder_2 < scalar_to_light)
                                        {
                                            #ifdef DEBUG_4_BLOCKED
                                                cerr << "plane_instance blocked light_container[" << i << "] for intersection at image point {x, y} {" << x << ", " << y << "}." << endl;
                                            #endif
                                            delete placeholder_2;
                                            continue;
                                        }
                                        delete placeholder_2;
                                    }
                                }
                                if (!sphere_container.empty())//spheres exist
                                {
                                    intersection_continue = false;

                                    for (j = 0; j < sphere_container.size(); ++j)
                                    {
                                        placeholder_2 = sphere_intersection(sphere_container[j], intersection_point, light_ray_direction);//only care about if there is an intersection

                                        if (placeholder_2[0] > 0.0f /*thus intersection exist*/ && ((SHADOW_BIAS < placeholder_2[1] && placeholder_2[1] < scalar_to_light) ||
                                           (placeholder_2[0] == 2.0f /*thus 2nd intersection exits*/ && SHADOW_BIAS < placeholder_2[2] && placeholder_2[2] < scalar_to_light)))//lower bound is greater than 0 to not block itself
                                        {
                                            #ifdef DEBUG_4_BLOCKED
                                                cerr << "sphere_container[" << j << "] blocked light_container[" << i << "] for intersection at image point {x, y} {" << x << ", " << y << "}." << endl;
                                            #endif
                                            intersection_continue = true;
                                            delete[] placeholder_2;
                                            break;
                                        }
                                        delete[] placeholder_2;
                                    }
                                    if (intersection_continue)
                                        continue;
                                }
                                if (mesh_instance.active)//mesh exists
                                {
                                    intersection_continue = false;

                                    for (j = 0; j < mesh_instance.vertices.size(); j += 3)//3 vertices make a triangle
                                    {
                                        placeholder_2 = triangle_intersection(mesh_instance.vertices[j], mesh_instance.vertices[j + 1], mesh_instance.vertices[j + 2],
                                                                              intersection_point, light_ray_direction);//only care about if there is an intersection
                                        if (placeholder_2 != nullptr)
                                        {
                                            if (SHADOW_BIAS < *placeholder_2 && *placeholder_2 < scalar_to_light)//lower bound is greater than 0 to not block itself
                                            {
                                                #ifdef DEBUG_4_BLOCKED
                                                    cerr << "mesh_instance.vertices[" << j << ", " << j + 2 << "] blocked light_container[" << i
                                                         << "] for intersection at image point {x, y} {" << x << ", " << y << "}." << endl;
                                                #endif
                                                intersection_continue = true;
                                                delete placeholder_2;
                                                break;
                                            }
                                            delete placeholder_2;
                                        }
                                    }
                                    if (intersection_continue)
                                        continue;
                                }

                                //illumination, has to not be skipped (continue) to be run
                                {
                                    #ifdef DEBUG_4_NOT_BLOCKED
                                        cerr << "Intersection at {x, y} {" << x << ", " << y << "} is illuminated by light_container[" << i << "]." << endl;
                                    #endif

                                    float diffuse_specular_dot_product [2] = {dot_product(light_ray_direction, intersection_point_normal), 0.0f};//for clamping, also to not repeat calculations

                                    //clamp
                                    if (diffuse_specular_dot_product[0] < 0.0f)
                                        diffuse_specular_dot_product[0] = 0.0f;
                                    {
                                        const float DOUBLE_DIFFUSE_DOT_PRODUCT = 2.0f * diffuse_specular_dot_product[0];
                                        for (j = 0; j < ARRAY_SIZE; ++j)
                                            diffuse_specular_dot_product[1] += (DOUBLE_DIFFUSE_DOT_PRODUCT * intersection_point_normal[j] - light_ray_direction[j]) * -orginal_ray_direction[j];//minus on orginal_ray_direction is to negate/reverse direction
                                    }
                                    //clamp
                                    if (diffuse_specular_dot_product[1] < 0.0f)
                                        diffuse_specular_dot_product[1] = 0.0f;
                                    //diffuse + specular
                                    for (j = 0; j < ARRAY_SIZE; ++j)
                                        output_image(x, y, j) += placeholder.first -> diffuse_colour[j] * diffuse_specular_dot_product[0] * light_container[i].diffuse_colour[j] +
                                        pow(diffuse_specular_dot_product[1], placeholder.first -> shininess) * placeholder.first -> specular_colour[j] * light_container[i].specular_colour[j];
                                }
                            }
                        }

                        for (i = 0; i < ARRAY_SIZE; ++i)
                        {
                            output_image(x, y, i) += placeholder.first -> ambient_colour[i];//add global ambient colour
                            output_image(x, y, i) = output_image(x, y, i) >= 1.0f ? 255.0f : output_image(x, y, i) * 255.0f;//scale colours and clamp
                            #ifdef DEBUG_4_NOT_BLOCKED
                                cerr << "Intersection at {x, y, channel} {" << x << ", " << y << ", " << i << "}, value: " << output_image(x, y, i) << endl;
                            #endif
                        }
                    }
                    #ifdef DEBUG_3_MISS
                        else //no intersection found for given pixel
                            cerr << "No intersection for image point {x, y} {" << x << ", " << y << "}" << endl;
                    #endif
                }
            }
        }

        //render
        {
            //time to uniquely create output file names
            const time_t RAW_TIME = time(nullptr);
            const struct tm * DATE_TIME = localtime(&RAW_TIME);
            #ifdef DEBUG_2
                cerr
                #ifdef ABSOLUTE_PATH
                    << ABSOLUTE_PATH
                #endif
                    << "Output/" << FILE_NAME << " " << DATE_TIME -> tm_year + 1900 << "-" << DATE_TIME -> tm_mon + 1 << "-" << DATE_TIME -> tm_mday
                    << " " << DATE_TIME -> tm_hour << "_" << DATE_TIME -> tm_min << "_" << DATE_TIME -> tm_sec << ".bmp";
            #endif
            output_image.save((
            #ifdef ABSOLUTE_PATH
                std::string(ABSOLUTE_PATH) +
            #endif
            "Output/" + FILE_NAME + " " + std::to_string(DATE_TIME-> tm_year + 1900) + "-" + std::to_string(DATE_TIME-> tm_mon + 1) + "-" + std::to_string(DATE_TIME-> tm_mday) +
            " " + std::to_string(DATE_TIME-> tm_hour) + "_" + std::to_string(DATE_TIME-> tm_min) + "_" + std::to_string(DATE_TIME-> tm_sec) + ".bmp").c_str());
            #ifdef DEBUG_2
                cimg_library::CImgDisplay image_display(output_image, "Output Image");
                while (!image_display.is_closed())
                {
                    image_display.wait();
                }
            #endif
        }
    }
}

#endif //RAY_TRACER_CPP
