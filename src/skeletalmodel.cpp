#include "skeletalmodel.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>

#include "starter2_util.h"
#include "vertexrecorder.h"

using namespace std;

SkeletalModel::SkeletalModel() {
    program = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program) {
        printf("Cannot compile program\n");
        assert(false);
    }
}

SkeletalModel::~SkeletalModel() {
    // destructor will release memory when SkeletalModel is deleted
    while (m_joints.size()) {
        delete m_joints.back();
        m_joints.pop_back();
    }

    glDeleteProgram(program);
}

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
    loadSkeleton(skeletonFile);

    m_mesh.load(meshFile);
    m_mesh.loadAttachments(attachmentsFile, (int)m_joints.size());

    computeBindWorldToJointTransforms();
    updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(const Camera& camera, bool skeletonVisible)
{
    // draw() gets called whenever a redraw is required
    // (after an update() occurs, when the camera moves, the window is resized, etc)

    m_matrixStack.clear();

    glUseProgram(program);
    updateShadingUniforms();
    if (skeletonVisible)
    {
        drawJoints(camera);
        drawSkeleton(camera);
    }
    else
    {
        // Tell the mesh to draw itself.
        // Since we transform mesh vertices on the CPU,
        // There is no need to set a Model matrix as uniform
        camera.SetUniforms(program, Matrix4f::identity());
        m_mesh.draw();
    }
    glUseProgram(0);
}

void SkeletalModel::updateShadingUniforms() {
    // UPDATE MATERIAL UNIFORMS
    GLfloat diffColor[] = { 0.4f, 0.4f, 0.4f, 1 };
    GLfloat specColor[] = { 0.9f, 0.9f, 0.9f, 1 };
    GLfloat shininess[] = { 50.0f };
    int loc = glGetUniformLocation(program, "diffColor");
    glUniform4fv(loc, 1, diffColor);
    loc = glGetUniformLocation(program, "specColor");
    glUniform4fv(loc, 1, specColor);
    loc = glGetUniformLocation(program, "shininess");
    glUniform1f(loc, shininess[0]);

    // UPDATE LIGHT UNIFORMS
    GLfloat lightPos[] = { 3.0f, 3.0f, 5.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightPos");
    glUniform4fv(loc, 1, lightPos);

    GLfloat lightDiff[] = { 120.0f, 120.0f, 120.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightDiff");
    glUniform4fv(loc, 1, lightDiff);
}

void SkeletalModel::loadSkeleton(const char* filename)
{
    // Load the skeleton from file here.
    const int MAX_BUFFER_SIZE = 4096;
    char buffer[MAX_BUFFER_SIZE];
//    std::cout << "made it to load skeleton" << std::endl;
    
    ifstream myfile (filename);

    if (myfile.is_open())
    {
        while(myfile.getline(buffer, MAX_BUFFER_SIZE)){
            Joint* joint = new Joint();
            stringstream ss(buffer);
            
            Vector3f v3;
            ss >> v3[0] >> v3[1] >> v3[2];//x, y, z
            Matrix4f transformMat = Matrix4f::identity();
//            float z_rotate = atan2(v3[1], v3[2]); //y,z
//            float y_rotate = atan2(v3[2], v3[1]); //z,y
//            transformMat = transformMat.rotateX(x_rotate);
//            transformMat = transformMat.rotateZ(y_rotate);
//            std::cout << "x_rotate " << x_rotate << std::endl;
            transformMat = Matrix4f::translation(v3);//*Matrix4f::rotateX(x_rotate)*Matrix4f::rotateY(y_rotate);
//            transformMat = transformMat.translation(v3);

//            transformMat = transformMat.rotateY(z_rotate);
//            transformMat.setCol(3, v4);
            joint->transform = transformMat;
            
            m_joints.push_back(joint);
            
            int parent;
            ss >> parent;
            if(parent == -1)
            {
                m_rootJoint = joint;
            }
            else
            {
                m_joints[parent]->children.push_back(joint);
                
            }
            
//            std::cout << "in the loop2" << std::endl;
//            std::cout << v3[0] << " " << v3[1] << " " << v3[2] << std::endl;
//            std::cout << "parent " << parent << std::endl;
        }
    }
//    std::cout << "size of m_joints " << m_joints.size() << std::endl;
//    for(Joint* joint : m_joints)
//    {
//        std::cout << "parent " <<joint->transform.getCol(3)[0] <<  " " << joint->transform.getCol(3)[1] <<  " " << joint->transform.getCol(3)[2] << std::endl;
//        for (Joint* child : joint->children)
//        {
//            std::cout << "\t child " <<child->transform.getCol(3)[0] <<  " " << child->transform.getCol(3)[1] <<  " " << child->transform.getCol(3)[2] << std::endl;
//        }
//    }
//  

}

void SkeletalModel::recursiveDrawDots(const Camera& camera, const Joint* joint)
{
//    std::cout << "recursiveDraw" << std::endl;
//    std::cout << joint->transform.getCol(3)[0] <<  " " << joint->transform.getCol(3)[1] <<  " " << joint->transform.getCol(3)[2] << std::endl;
    // translate from top of stack
    m_matrixStack.push(joint->transform);
    Matrix4f M = m_matrixStack.top();
    // update transformation uniforms
    camera.SetUniforms(program, M);
    // draw
    drawSphere(0.025f, 12, 12);
    for (Joint* child : joint->children)
    {
        recursiveDrawDots(camera, child);
        float distance = (child->transform.getCol(3)).abs();
    }
//    std::cout << "recursiveDraw2" << std::endl;
    m_matrixStack.pop();
}

void SkeletalModel::recursiveDrawCylinders(const Camera& camera, const Joint* joint, const Joint* parent)
{
//    std::cout << "recursiveDraw" << std::endl;
//    std::cout << joint->transform.getCol(3)[0] <<  " " << joint->transform.getCol(3)[1] <<  " " << joint->transform.getCol(3)[2] << std::endl;
    
    // translate from top of stack
    m_matrixStack.push(parent->transform);
    Vector3f translationVector = joint->transform.getCol(3).xyz();
//    translationVector = (parent->transform*joint->transform.getCol(3)).xyz();
    Vector3f otherVector = translationVector;
    otherVector[0] = translationVector[0] - 1;
    Vector3f translationVectorX = Vector3f::cross(translationVector, otherVector);
    Vector3f translationVectorZ = Vector3f::cross(translationVectorX, translationVector);
    
    Vector3f translationVectorNormalized = translationVector;
    Vector3f translationVectorNormalizedX = translationVectorX;
    Vector3f translationVectorNormalizedZ = translationVectorZ;
    
    translationVectorNormalized.normalize();
    translationVectorNormalizedX.normalize();
    translationVectorNormalizedZ.normalize();

//    std::cout << translationVectorNormalized[0] << " " << translationVectorNormalized[1] << " " << translationVectorNormalized[2] << std::endl;
//    
//    std::cout << translationVectorNormalizedX[0] << " " << translationVectorNormalizedX[1] << " " << translationVectorNormalizedX[2] << std::endl;
//    
//    std::cout << translationVectorNormalizedZ[0] << " " << translationVectorNormalizedZ[1] << " " << translationVectorNormalizedZ[2] << std::endl;
//    
    Matrix4f rotateMat = Matrix4f(
                                  Vector4f(translationVectorNormalizedX, 0),
//                                  Vector4f(1,0,0,0),
                                  Vector4f(translationVectorNormalized, 0),
                                  Vector4f(translationVectorNormalizedZ, 0),
//                                  Vector4f(0,0,1,0),
                                  Vector4f(0,0,0,1)
                                  );
    
    
//    Matrix4f rotateMat = Matrix4f::identity();
//    rotateMat.setCol(1, Vector4f(translationVectorNormalized, 0));
    
    m_matrixStack.push(rotateMat);
    Matrix4f M = m_matrixStack.top();
    float distance = translationVector.abs();
//    std::cout << "distance " << distance << std::endl;
    camera.SetUniforms(program, M);
    drawCylinder(6, 0.02f, distance);
    m_matrixStack.pop();

//    m_matrixStack.pop();
    for (Joint* child : joint->children)
    {
        recursiveDrawCylinders(camera, child, joint);
    }
//    std::cout << "recursiveDraw2" << std::endl;
    m_matrixStack.pop();
}

void SkeletalModel::drawJoints(const Camera& camera)
{
    // Draw a sphere at each joint. You will need to add a recursive
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawSphere( 0.025f, 12, 12 )
    // to draw a sphere of reasonable size.
    //
    // You should use your MatrixStack class. A function
    // should push it's changes onto the stack, and
    // use stack.pop() to revert the stack to the original
    // state.
//    std::cout << "draw joints" << std::endl;
//    std::cout << "draw joints2" << std::endl;
//    Matrix4f M = Matrix4f::identity() * Matrix4f::translation(+0.5f, +0.5f, -0.5f);
    // update transformation uniforms
//    camera.SetUniforms(program, M);
    // draw
//    drawSphere(0.25f, 12, 12);
//    drawCylinder(6, 0.02f, 1);
    recursiveDrawDots(camera, m_rootJoint);
}

void SkeletalModel::drawSkeleton(const Camera& camera)
{
    // Draw cylinders between the joints. You will need to add a recursive 
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawCylinder(6, 0.02f, <height>);
    // to draw a cylinder of reasonable diameter.

    // you can use the stack with push/pop like this
    // m_matrixStack.push(Matrix4f::translation(+0.6f, +0.5f, -0.5f))
    // camera.SetUniforms(program, m_matrixStack.top());
    // drawCylinder(6, 0.02f, 0.2f);
    // callChildFunction();
    // m_matrixStack.pop();
    // translate from top of stack
    for (Joint* child : m_rootJoint->children)
    {
        recursiveDrawCylinders(camera, child, m_rootJoint);
    }
//    std::cout << "recursiveDraw2" << std::endl;
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
//    std::cout << "setJointTransform " << jointIndex << " " << rX << " " << rY << " " << rZ << std::endl;
    Matrix4f rotateMat = Matrix4f::identity();
//    Vector3f translationVector = Vector3f(rX, rY, rZ);
//    Vector3f translationVectorNormalized = translationVector;
//    rotateMat.setCol(1, Vector4f(translationVectorNormalized, 0));
    m_joints[jointIndex]->transform.setSubmatrix3x3(0, 0, Matrix3f::rotateX(rX)
                                                        * Matrix3f::rotateY(rY)
                                                        * Matrix3f::rotateZ(rZ));
}

void SkeletalModel::recursiveBindWorldToJointTransforms(Joint* joint, Matrix4f parentTransform)
{
    Matrix4f jointTransform = parentTransform*joint->transform;
    joint->bindWorldToJointTransform = jointTransform.inverse();
    for (Joint* child : joint->children)
    {
        recursiveBindWorldToJointTransforms(child, jointTransform);
    }
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
    // 2.3.1. Implement this method to compute a per-joint transform from
    // world-space to joint space in the BIND POSE.
    //
    // Note that this needs to be computed only once since there is only
    // a single bind pose.
    //
    // This method should update each joint's bindWorldToJointTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
    recursiveBindWorldToJointTransforms(m_rootJoint, Matrix4f::identity());
}

void SkeletalModel::recursiveCurrentJointToWorldTransforms(Joint* joint, Matrix4f parentTransform)
{
    Matrix4f jointTransform = parentTransform*joint->transform;
    joint->currentJointToWorldTransform = jointTransform;
    for (Joint* child : joint->children)
    {
        recursiveCurrentJointToWorldTransforms(child, jointTransform);
    }
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
    // 2.3.2. Implement this method to compute a per-joint transform from
    // joint space to world space in the CURRENT POSE.
    //
    // The current pose is defined by the rotations you've applied to the
    // joints and hence needs to be *updated* every time the joint angles change.
    //
    // This method should update each joint's currentJointToWorldTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
    recursiveCurrentJointToWorldTransforms(m_rootJoint, Matrix4f::identity());
}

void SkeletalModel::updateMesh()
{
    // 2.3.2. This is the core of SSD.
    // Implement this method to update the vertices of the mesh
    // given the current state of the skeleton.
    // You will need both the bind pose world --> joint transforms.
    // and the current joint --> world transforms.
    for(int i =0; i<(int)m_mesh.bindVertices.size(); i++)
    {
        Vector4f vertex;
        Vector4f bindVertex = Vector4f(m_mesh.bindVertices[i], 1);
        for(int j=0; j<(int)m_joints.size(); j++)
        {
            vertex = vertex
                    + m_mesh.attachments[i][j]
                    * m_joints[j]->currentJointToWorldTransform
                    * m_joints[j]->bindWorldToJointTransform
                    * bindVertex;
        }
        m_mesh.currentVertices[i] = vertex.xyz();
    }
}

