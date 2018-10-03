#include "mesh.h"
#include <iostream>
#include <fstream>
#include <string>

#include "vertexrecorder.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 4.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
    const int MAX_BUFFER_SIZE = 4096;
    char buffer[MAX_BUFFER_SIZE];
    
    ifstream myfile (filename);
    
    if (myfile.is_open())
    {
        while(myfile.getline(buffer, MAX_BUFFER_SIZE)){
            stringstream ss(buffer);
            string s;
            ss >> s;
            if (s == "v") {
                Vector3f v;
                ss >> v[0] >> v[1] >> v[2];
                bindVertices.push_back(v);
            }
            else if (s == "f") {
                vector<unsigned> v;
                string token;
                string face;
                int int_token;
                while(ss >> face){
                    istringstream sf(face);
                    while(std::getline(sf, token, '/')) {
                        istringstream si(token);
                        si >> int_token;
                        v.push_back(int_token);
                    }
                }
                faces.push_back(Tuple3u(v[0], v[1], v[2]));
            }
        }
    }
    std::cout << "faces size " << faces.size() << std::endl;
    std::cout << "bindVertices size " << bindVertices.size() << std::endl;
    
	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// 4.2 Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
    VertexRecorder rec;
    int vecf_size = faces.size();
    for(int i = 0; i < vecf_size; i++){
        Tuple3u v = faces[i];
        Vector3f v1 = currentVertices[v[1]-1] - currentVertices[v[0]-1];
        Vector3f v2 = currentVertices[v[0]-1] - currentVertices[v[2]-1];
        Vector3f normal = -Vector3f::cross(v1, v2);
//        std::cout << "made it here " << std::endl;
        rec.record(currentVertices[v[0]-1], normal);//a
        rec.record(currentVertices[v[1]-1], normal);//d
        rec.record(currentVertices[v[2]-1], normal);//g
    }
    rec.draw();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 4.3. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
    const int MAX_BUFFER_SIZE = 4096;
    char buffer[MAX_BUFFER_SIZE];
    
    ifstream myfile (filename);
    
    if (myfile.is_open())
    {
        while(myfile.getline(buffer, MAX_BUFFER_SIZE)){
            stringstream ss(buffer);
            float w;
            vector<float> weights;
            weights.push_back(0);
            while(ss >> w)
            {
                weights.push_back(w);
            }
            attachments.push_back(weights);
        }
    }
//    std::cout << "attachments size " << attachments.size() << std::endl;
    // make a copy of the bind vertices as the current vertices
    
}
