// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef Sequence_H
#define Sequence_H

#include <QDebug>

#include <cameraSet.h>
#include <animation.h>

class Sequence
{
public:
    Sequence();
    Sequence(
                            //CAMERASET
                              QString INPUT_PATH_CAMERA_SET_IN,

                            //MODEL
                              QString INPUT_BasePath_OutPut_IN,
                              QString INPUT_dynamicStringPart_IN,
                              QString INPUT_EXTENSSS_Mesh_IN,
                              QString INPUT_EXTENSSS_Skeleton_IN,
                              QString INPUT_EXTENSSS_Skin_IN,
                              QString INPUT_PATH_MODELS_INFO_IN,

                            //ANIMATION
                              QString PATH_OutputBase_IN,
                              QString PATH_FolderName_INPUT_IN,
                              QString INPUT_EXTENSSS_Motion_IN,
                              QString PATH_INDEX_BOUNDS_IN,

                            //SEQUENCE
                              QString PATH_INDEX_BOUNDS_INNN,
                              QString RadioSequenceID_String_IN,

                              bool printEnabled
            );


    CameraSet          cameraSet;
    QVector<Animation> posedAnimations;

    int totalAllignedFrames;
    int motionOffset;

    int currentFrameNumber_Source;
    int currentFrameNumber_Target;

    int     sequenceID;
    QString sequenceID_String;

    void updateFrameNumber( int currentFrameNumber_Source_IN_noOffset, int currentFrameNumber_Target_IN_noOffset );

    void appendNewPosedAnimation( Animation posedAnimation_IN );

    void transform_All_Model_MESHES( const int& animID, const Eigen::Matrix4f& transf_4x4, const bool& colorPerANIM_TRUE__colorPerModel_FALSE );
    void transform_All_Model_MESHES(                                                       const bool& colorPerANIM_TRUE__colorPerModel_FALSE );
    void transform_All_Model_SKELETONS();

    void printAnimationINFO();

};

#endif // Sequence_H
