//Modified to not use glm and to not bother with uvs and normals stuff as they are not used.

#ifndef OBJLOADER_MODIFIED_H_
#define OBJLOADER_MODIFIED_H_

//#include "glm.hpp"
#include <cstring>
#include <vector>
#include <string>
#include <stdlib.h>
#include <array>

bool loadOBJ(
    const char * path,
    std::vector</*glm::vec3*/std::array<float, 3>> & out_vertices/*,
    std::vector</*glm::vec3*//*float [3]> & out_normals,
    std::vector<glm::vec2int [2]> & out_uvs*/) {

    std::vector<int> vertexIndices/*, uvIndices, normalIndices*/;
    std::vector</*glm::vec3*/std::array<float, 3>> temp_vertices;
    //std::vector</*glm::vec2*/int [2]> temp_uvs;
    //std::vector</*glm::vec3*/ float [3]> temp_normals;

    FILE * file;
    int _errno = fopen_s(&file, path, "r");
    if (_errno != 0) {
        printf("Impossible to open the file ! Are you in the right path ?\n");
        getchar();
        return false;
    }

    while (1) {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf_s(file, "%s", lineHeader, 128);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader

        if (strcmp(lineHeader, "v") == 0) {
            /*glm::vec3*/std::array<float, 3> vertex;
            fscanf_s(file, "%f %f %f\n", &vertex[0]/*.x*/, &vertex[1]/*.y*/, &vertex[2]/*.z*/);
            temp_vertices.push_back(vertex);
        }
        else if (strcmp(lineHeader, "vt") == 0) {
            /*glm::vec2*/float uv [2];
            fscanf_s(file, "%f %f\n", &uv[0]/*.x*/, &uv[1]/*.y*/);
            //uv.y = -uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
            //temp_uvs.push_back(uv);
        }
        else if (strcmp(lineHeader, "vn") == 0) {
            /*glm::vec3*/float normal [3];
            fscanf_s(file, "%f %f %f\n", &normal[0]/*.x*/, &normal[1]/*.y*/, &normal[2]/*.z*/);
            //temp_normals.push_back(normal);
        }
        else if (strcmp(lineHeader, "f") == 0) {
            int vertexIndex[3], uvIndex[3], normalIndex[3];
//            bool uv = true;
//            bool norm = true;
            char line[128];
            fgets(line, 128, file);
            //vertex, uv, norm
            int matches = sscanf_s(line, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
            if (matches != 9) {
                //vertex, norm
                matches = sscanf_s(line, "%d//%d %d//%d %d//%d\n", &vertexIndex[0], &normalIndex[0], &vertexIndex[1], &normalIndex[1], &vertexIndex[2], &normalIndex[2]);
                if (matches != 6) {
                    //vertex, uv
                    matches = sscanf_s(line, "%d/%d %d/%d %d/%d\n", &vertexIndex[0], &uvIndex[0], &vertexIndex[1], &uvIndex[1], &vertexIndex[2], &uvIndex[2]);
                    if (matches != 6) {
                        //vertex
                        matches = sscanf_s(line, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);
                        if (matches != 6) {
                            printf("File can't be read by our simple parser. 'f' format expected: d/d/d d/d/d d/d/d || d/d d/d d/d || d//d d//d d//d\n");
                            printf("Character at %d", ftell(file));
                            return false;
                        }
//                        uv, norm = false;
                    }
//                    else {
//                        norm = false;
//                    }
                }
//                else {
//                    uv = false;
//                }
            }
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
//            if (norm) {
//                normalIndices.push_back(normalIndex[0]);
//                normalIndices.push_back(normalIndex[1]);
//                normalIndices.push_back(normalIndex[2]);
//            }
//            if (uv) {
//                uvIndices.push_back(uvIndex[0]);
//                uvIndices.push_back(uvIndex[1]);
//                uvIndices.push_back(uvIndex[2]);
//            }
        }
        else {
            char clear[1000];
            fgets(clear, 1000, file);
        }

    }
    //std::cout << "Vertex indices: " << vertexIndices.size() << std::endl;
    //std::cout << "UV indices: " << uvIndices.size() << std::endl;
    //std::cout << "Normal indices: " << normalIndices.size() << std::endl;
    // For each vertex of each triangle
    for (unsigned int i = 0; i < vertexIndices.size(); i++) {
//        if (uvIndices.size() != 0) {
//            if (i < uvIndices.size()) {
//                unsigned int uvIndex = abs(uvIndices[i]);
//                glm::vec2 uv = temp_uvs[uvIndex - 1];
//                out_uvs.push_back(uv);
//            }
//        }
//        if (normalIndices.size() != 0) {
//            if (i < normalIndices.size()) {
//                unsigned int normalIndex = abs(normalIndices[i]);
//                /*glm::vec3*/float normal [3] = {temp_normals[normalIndex - 1]};
//                out_normals.push_back(normal);
//            }
//        }

        unsigned int vertexIndex = abs(vertexIndices[i]);
        /*glm::vec3*/std::array<float, 3> vertex = temp_vertices[vertexIndex - 1];
        out_vertices.push_back(vertex);
    }

    return true;
}

#endif//OBJLOADER_MODIFIED_H_
