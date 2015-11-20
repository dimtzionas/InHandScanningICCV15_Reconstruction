// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "sequence.h"

Sequence::Sequence(){}
Sequence::Sequence(
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
                  )
{

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        cameraSet = CameraSet( INPUT_PATH_CAMERA_SET_IN );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        Animation animSource = Animation(
                                              //MODEL
                                                INPUT_BasePath_OutPut_IN,
                                                INPUT_dynamicStringPart_IN,
                                                INPUT_EXTENSSS_Mesh_IN,
                                                INPUT_EXTENSSS_Skeleton_IN,
                                                INPUT_EXTENSSS_Skin_IN,
                                                INPUT_PATH_MODELS_INFO_IN,

                                              //ANIMATION
                                                PATH_OutputBase_IN,
                                                PATH_FolderName_INPUT_IN,
                                                INPUT_EXTENSSS_Motion_IN,
                                                PATH_INDEX_BOUNDS_IN,
                                                "Source_Animation",

                                                printEnabled
                                        );

        Animation animTarget = Animation(
                                              //MODEL
                                                INPUT_BasePath_OutPut_IN,
                                                INPUT_dynamicStringPart_IN,
                                                INPUT_EXTENSSS_Mesh_IN,
                                                INPUT_EXTENSSS_Skeleton_IN,
                                                INPUT_EXTENSSS_Skin_IN,
                                                INPUT_PATH_MODELS_INFO_IN,

                                              //ANIMATION
                                                PATH_OutputBase_IN,
                                                PATH_FolderName_INPUT_IN,
                                                INPUT_EXTENSSS_Motion_IN,
                                                PATH_INDEX_BOUNDS_IN,
                                                "Target_Animation",

                                                printEnabled
                                        );

        posedAnimations.append( animSource );
        posedAnimations.append( animTarget );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        currentFrameNumber_Source = 0;
        currentFrameNumber_Target = 0;

        sequenceID_String = RadioSequenceID_String_IN;
        sequenceID        = sequenceID_String.toInt() - 1;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        QFile myFile ( PATH_INDEX_BOUNDS_INNN );

        myFile.open(QIODevice::ReadOnly);

        if( !myFile.isOpen() )
        {
            qDebug() << "\n\n\n   Sequence::Sequence - ERROR, unable to open **" << PATH_INDEX_BOUNDS_INNN << "** for IndexCheat Input \n\n\n";
            return;
        }

        QTextStream myStream(&myFile);

        QString dummyDescr;

        myStream >> dummyDescr;       myStream >> totalAllignedFrames;      posedAnimations[0].totalAllignedFrames = totalAllignedFrames;
        myStream >> dummyDescr;       myStream >> motionOffset;             posedAnimations[0].motionOffset        = motionOffset;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////
        /////////////////////////////////////////
        if (printEnabled)   printAnimationINFO();
        /////////////////////////////////////////
        /////////////////////////////////////////

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Sequence::printAnimationINFO()
{
    qDebug() << "\n";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "################################################################################################################################################################# Sequence of Animations";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "";
    qDebug() << "Sequence::printAnimationINFO \t totalAllignedFrames - " << totalAllignedFrames;
    qDebug() << "Sequence::printAnimationINFO \t motionOffset        - " << motionOffset;
    qDebug() << "";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "\n";
}


void Sequence::appendNewPosedAnimation( Animation posedAnimation_IN )
{
    posedAnimations.append( posedAnimation_IN );
}



void Sequence::transform_All_Model_MESHES( const int& animID, const Eigen::Matrix4f& transf_4x4, const bool& colorPerANIM_TRUE__colorPerModel_FALSE )
{
    posedAnimations[animID].transform_ModelSet_MESHES( transf_4x4 );

    posedAnimations[animID].convert_MESH_2_PCL( animID, colorPerANIM_TRUE__colorPerModel_FALSE );
}



void Sequence::transform_All_Model_MESHES( const bool& colorPerANIM_TRUE__colorPerModel_FALSE )
{
    posedAnimations[0].transform_ModelSet_MESHES( currentFrameNumber_Source );      posedAnimations[0].convert_MESH_2_PCL( 0, colorPerANIM_TRUE__colorPerModel_FALSE );
    posedAnimations[1].transform_ModelSet_MESHES( currentFrameNumber_Target );      posedAnimations[1].convert_MESH_2_PCL( 1, colorPerANIM_TRUE__colorPerModel_FALSE );
}


void Sequence::transform_All_Model_SKELETONS()
{
    posedAnimations[0].touchingJoints.clear();
    posedAnimations[1].touchingJoints.clear();

    posedAnimations[0].transform_ModelSet_SKELETONS( currentFrameNumber_Source );
    posedAnimations[1].transform_ModelSet_SKELETONS( currentFrameNumber_Target );
}



void Sequence::updateFrameNumber( int currentFrameNumber_Source_IN_noOffset, int currentFrameNumber_Target_IN_noOffset )
{
    currentFrameNumber_Source = currentFrameNumber_Source_IN_noOffset;
    currentFrameNumber_Target = currentFrameNumber_Target_IN_noOffset;

    if (currentFrameNumber_Source <  0   ||   currentFrameNumber_Source >= totalAllignedFrames)     {       currentFrameNumber_Source = 0;      qDebug() << "\n\n\n\n\n\n" << "Sequence::updateFrameNumber - pt1" << "\n\n\n\n\n\n";      }
    if (currentFrameNumber_Target <  0   ||   currentFrameNumber_Target >= totalAllignedFrames)     {       currentFrameNumber_Target = 0;      qDebug() << "\n\n\n\n\n\n" << "Sequence::updateFrameNumber - pt2" << "\n\n\n\n\n\n";      }
}
