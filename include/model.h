// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef MODEL_H
#define MODEL_H

#include <QDebug>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QFile>

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct BoneStruct
{
        QString         name;
        double          length;
        int             addressSeListaBones;
        int             addressSeListaSKINNING;

        QString         fatherName;
        int             fatherAddress;

        QVector<int>    childrenVector;

        bool            isUsedForSkinning;
};

struct SkeletonStruct
{
        QVector< BoneStruct >  bones;
                 BoneStruct    tempBone;
                 BoneStruct    currentBone;
                 BoneStruct    startingBone;
};

struct skinningBoneStruct
{
        QString                 name;

        Eigen::MatrixXd         bone_Start_4x1;
        Eigen::MatrixXd         bone_Leaff_4x1;

        int                     mainlyInfluenced_TotalVertices;
        QVector<int>            mainlyInfluenced_VertexIDs;

        pcl::PointXYZ           bone_Start_PclPointXYZ;
        pcl::PointXYZ           bone_Leaff_PclPointXYZ;
        bool                    bone_Start_isInTouch;
        bool                    bone_Leaff_isInTouch;
        int                     bone_Start_RRR;
        int                     bone_Start_GGG;
        int                     bone_Start_BBB;
        int                     bone_Leaff_RRR;
        int                     bone_Leaff_GGG;
        int                     bone_Leaff_BBB;
        int                     addressSeListaTouchingJoints_Start;
        int                     addressSeListaTouchingJoints_Leaff;
};

struct SkinningBonesSkeletonStruct
{
        QVector< skinningBoneStruct >   skinningBones;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct TriangleStruct
{
        int vertexID_1;
        int vertexID_2;
        int vertexID_3;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct DominantSkinningBone
{
    int    skinningBoneID;
    double skinningWeight;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct MeshStruct
{
        QVector< DominantSkinningBone >     vertices_dominantSkinningBone;
        QVector< Eigen::Vector3d      >     vertices;
        QVector< Eigen::Vector3d      >     verticesOLD;
        QVector< Eigen::Vector3d      >     verticesWeighted;
        QVector< Eigen::Vector3d      >     normals_Vertices;
        QVector< TriangleStruct       >     triangles;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SkinWegitsForVertexStruct
{
        QVector< float >      skinWeights;
        QVector< float >      skinWeights_Thresholded;
};

struct SkinStruct
{
        QVector< SkinWegitsForVertexStruct > skinnedVertices;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class Model
{
public:
    Model();
    Model(      QString INPUT_BasePath,
                QString INPUT_dynamicStringPart,
                QString INPUT_fileNameOhneExtension,
                QString INPUT_EXTENSSS_Mesh,
                QString INPUT_EXTENSSS_Skeleton,
                QString INPUT_EXTENSSS_Skin
         );
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    bool has_OnlySkin;
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////

    void readMesh_OFF(   QString myFileString_Mesh     );
    void readSkeleton(   QString myFileString_Skeleton );
    void readSkin_TXT(   QString myFileString_Skin     );

    void print_SkeletonBones();
    void print_Mesh_Vertices();
    void print_Mesh_Triangles();
    void print_SkinningWeights();
    void print_SkinningBones();

    void compute_NORMALS();
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    QString name;
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr   PCL;
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    bool isRenderable;
    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////
    int totalSkeletonFrames;
    int totalBones;
    int totalVertices;
    int totalTriangles;
    int totalSkinningBones;
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    SkeletonStruct                  skeleton;
    SkinningBonesSkeletonStruct     skinningSkeleton;
    MeshStruct                      mesh;
    SkinStruct                      skin;
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    QString INPUT_myFileString_Mesh;
    QString INPUT_myFileString_Skeleton;
    QString INPUT_myFileString_Skin;
    QString INPUT_myFileString_Motion;
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    void create_Lookup_SkinningBone_to_Bones_ID();
    void   test_Lookup_SkinningBone_to_Bones_ID();

    void create_Lookup_KinematicChain_AllBones();
    void  print_Lookup_KinematicChain_AllBones();

    void create_Lookup_KinematicChain_SkinnBones_2_SkinnBones();
    void  print_Lookup_KinematicChain_SkinnBones_2_SkinnBones();

    void print_SkinningBoneNames();

    void create_thresholded_SkinningWeights();
    void  print_thresholded_SkinningWeights();

    void test_SkinningWeihgts_sum_to_1();

    int  find_Dominant_SkinningBone( const int vertexID ); // returns skkk
    void find_Dominant_SkinningBone( const int vertexID, int &dominant_SkinningBoneID, double &dominant_SkinningBoneCONF );

    void create_VerticesMainlyInfluenced_PerSkinningBone();
    void  print_VerticesMainlyInfluenced_PerSkinningBone();

    void create_Vertices_DominantSkinningBone();
    void  print_Vertices_DominantSkinningBone();

    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////

    bool    vertexBelongsToEndEffector( int vertexID );
    void    vertexBelongsToEndEffector_TEST();
    QString vertexBelongsToEndEffector_TEST( int vertexID );

    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////

    QVector< QVector< int > >   lookup_Influence_AllBones;
    QVector< QVector< int > >   lookup_Influence_SkinnBones_2_SkinnBones;          // myModel.lookup_Influence_SkinnBones_2_SkinnBones[ child ][ parent ]
             QVector< int >     lookup_SkinningBone_to_Bones_ID;

    /////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////

    void writeMesh_OFF___POSE_NULL(   QString Filename );
    void writeMesh_OFF___POSE_CUSTOM( QString Filename );

};

#endif // MODEL_H
