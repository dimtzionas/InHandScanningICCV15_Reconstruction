// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <modelSET.h>

#include <iomanip>   // std::setprecision()


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

struct PosedBone
{
    QString name;

    Eigen::MatrixXd R;
    Eigen::MatrixXd T;
    Eigen::MatrixXd RT_3x4;
    Eigen::MatrixXd RT_4x4;
};

struct PosedSkeleton
{
    QVector< PosedBone >   bones;
};

struct movingModel
{
    QVector< PosedSkeleton >    motionFrames;

    QString PATH_complete_Motion_INput;
    QString PATH_complete_Motion_OUTput;
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

struct TouchingJoint
{
    Eigen::Vector3f joint_Eigen_Vec3f;
    pcl::PointXYZ   joint_PclPointXYZ;
    QString         name_Bone;
    int             addressInSkinningBones;
    bool            isEndEffector;
    double          distanceToPCL_Signed_Joint2PCL;
    double          distanceToPCL_Signed_Joint2PCL_EF;
    int             modelID;
};

struct TouchingVertex
{
    Eigen::Vector3f        vertex_Eigen_Vec3f;
    pcl::PointXYZRGBNormal vertex_XYZRGBNormal;
    int                    addressInVertices;
    int                    dominantSkinningBoneID;
    QString                dominantSkinningBoneName;
    double                 distanceToPrimitive_Signed;
    int                    modelID;
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////



class Animation
{

public:
    Animation();
    Animation(
                  //MODEL
                   QString INPUT_BasePath_OutPut_IN,
                   QString INPUT_dynamicStringPart_IN,
                   QString INPUT_EXTENSSS_Mesh_IN,
                   QString INPUT_EXTENSSS_Skeleton_IN,
                   QString INPUT_EXTENSSS_Skin_IN,
                   QString INPUT_PATH_MODELS_INFO,

                 //ANIM
                   QString PATH_OutputBase_IN,
                   QString PATH_FolderName_INPUT_IN,
                   QString INPUT_EXTENSSS_Motion_IN,
                   QString PATH_INDEX_BOUNDS_IN,
                   QString name_IN,

                    bool printEnabled
             );

    /////////////////////////////////////////////////////////////////////////////////////

    QString name;

    ModelSet modelSet;

    int motionOffset;
    int totalAllignedFrames;

    /////////////////////////////////////////
    /////////////////////////////////////////
    QVector< movingModel >      movingModels;
    /////////////////////////////////////////
    /////////////////////////////////////////

    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    QVector< TouchingJoint >    touchingJoints;
             TouchingJoint      touchingJointTMP;
    /////////////////////////////////////////////////////////
    QVector< TouchingVertex >   touchingVertices;
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////

    void convert_MESH_2_PCL( const int animNumb, const bool colorPerANIM_TRUE__colorPerModel_FALSE );

    void transform_ModelSet_MESHES( const Eigen::Matrix4f& transf_4x4 );
    void transform_ModelSet_MESHES(    int frameNumber_XwrisOffset );
    void transform_ModelSet_SKELETONS( int frameNumber_XwrisOffset );

    void calculate_Bones_using_RT( int modelID, int frameNumber_MeOffset, int currrAdd );

    /////////////////////////////////////////////////////////////////////////////////////

    void  readCheatIndexTXT( QString PATH_INDEX_BOUNDS_IN );
    void printCheatIndexTXT();

    /////////////////////////////////////////////////////////////////////////////////////

    void printAnimationID();

    /////////////////////////////////////////////////////////////////////////////////////
    void poseMatrices_Clear();
    void poseMatrices_LoadFromFile( QString shouldPrint );
    void poseMatrices_print();
    void poseMatrices_print_minimal();
    /////////////////////////////////////////////////////////////////////////////////////

    void construct__RT_3x4__RT_4x4__basedOn__R_T__1bone( int modelID, int fr, int b );

    /////////////////////////////////////////////////////////////////////////////////////

    void TouchingVertices_TRASNFORM( Eigen::Matrix4f transformationMatrix4f );
    void TouchingVertices_PRINT();
    void TouchingVertices_PRINT_DBG(                              const QVector< TouchingVertex > &myTouchingVertices );
    int  TouchingVertices_READ_DBG(   QString FULL_PATH_READ,           QVector< TouchingVertex > &myTouchingVertices );
    int  TouchingVertices_READ(       QString FULL_PATH_READ );
    void TouchingVertices_WRITE(      QString FULL_PATH_SAVE );


private:
    void TouchingVertices_PRINT(                                  const QVector< TouchingVertex > &myTouchingVertices );
    int  TouchingVertices_READ(       QString FULL_PATH_READ,           QVector< TouchingVertex > &myTouchingVertices );

};





#endif // SEQUENCE_H





