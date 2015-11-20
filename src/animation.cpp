// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "animation.h"



Animation::Animation()
{
}



Animation::Animation(
                          //MODELSET
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
                           QString animationNAME_IN,

                           bool printEnabled
                    )
{



        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////
        name = animationNAME_IN;
        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////

                                                                                                                            ///////////////////////////////////////
                                                                                                                            ////  PRINTERS  ///////////////////////
                                                                                                                            ///////////////////////////////////////
                                                                                                                            if (printEnabled)   printAnimationID();
                                                                                                                            ///////////////////////////////////////
                                                                                                                            ///////////////////////////////////////

        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////
        modelSet = ModelSet(    INPUT_BasePath_OutPut_IN,
                                INPUT_dynamicStringPart_IN,
                                INPUT_EXTENSSS_Mesh_IN,
                                INPUT_EXTENSSS_Skeleton_IN,
                                INPUT_EXTENSSS_Skin_IN,
                                INPUT_PATH_MODELS_INFO,
                                printEnabled  );

        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////
        movingModels.resize( modelSet.totalModels );
        movingModels.squeeze();
        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////

        if      (PATH_OutputBase_IN.       endsWith("/") == false )                                                         PATH_OutputBase_IN       += "/";
        if      (PATH_FolderName_INPUT_IN. endsWith("/") == false && PATH_FolderName_INPUT_IN.contains("RGBD_") ==false )   PATH_FolderName_INPUT_IN += "/";
        else if (PATH_FolderName_INPUT_IN. endsWith("/") == false && PATH_FolderName_INPUT_IN.contains("RGBD_") ==true  )   PATH_FolderName_INPUT_IN +=                                                                    "__OK/";
        else if (PATH_FolderName_INPUT_IN. endsWith("/") == true  && PATH_FolderName_INPUT_IN.contains("RGBD_") ==true  )   PATH_FolderName_INPUT_IN  = PATH_FolderName_INPUT_IN.left(PATH_FolderName_INPUT_IN.size()-1) + "__OK/";

        ///////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////

        for (int modelID=0; modelID<modelSet.totalModels; modelID++)
        {
            movingModels[modelID].PATH_complete_Motion_INput = PATH_OutputBase_IN + PATH_FolderName_INPUT_IN  + modelSet.models[modelID].name + INPUT_EXTENSSS_Motion_IN;

            qDebug() << "Animation::Animation - modelID" << modelID << "- PATH_complete_Motion_INput - " << movingModels[modelID].PATH_complete_Motion_INput;
            qDebug() << "Animation::Animation - PATH_OutputBase_IN       - " << PATH_OutputBase_IN;
            qDebug() << "Animation::Animation - PATH_FolderName_INPUT_IN - " << PATH_FolderName_INPUT_IN;
        }
        qDebug() << "";


        //////////////////////////////////////////
        //////////////////////////////////////////
        readCheatIndexTXT( PATH_INDEX_BOUNDS_IN );
        //////////////////////////////////////////
        //////////////////////////////////////////
        poseMatrices_LoadFromFile( "readInput" );
        //////////////////////////////////////////
        //////////////////////////////////////////

                                                                                                                            ///////////////////////////////////
                                                                                                                            ////  PRINTERS  ///////////////////
                                                                                                                            ///////////////////////////////////
                                                                                                                            if (printEnabled)
                                                                                                                            {
                                                                                                                                printCheatIndexTXT();
                                                                                                                                poseMatrices_print_minimal();
                                                                                                                              //poseMatrices_print();
                                                                                                                            }
                                                                                                                            ///////////////////////////////////
                                                                                                                            ///////////////////////////////////

}


void Animation::printAnimationID()
{
    qDebug() << "\n";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "################################################################################################################################################################# " << name;
    qDebug() << "#################################################################################################################################################################";
    qDebug() << "#################################################################################################################################################################";
}



void Animation::readCheatIndexTXT( QString PATH_INDEX_BOUNDS_IN )
{

        QFile myFile ( PATH_INDEX_BOUNDS_IN );

        myFile.open(QIODevice::ReadOnly);

        if( !myFile.isOpen() )
        {
            qDebug() << "\n\n\n   Animation::readCheatIndexTXT - ERROR, unable to open **" << PATH_INDEX_BOUNDS_IN << "** for IndexCheat Input \n\n\n";
            return;
        }

        QTextStream myStream(&myFile);

        int         dummyValue;
        QString     dummyDescr;
        myStream >> dummyDescr;       myStream >> totalAllignedFrames;
        myStream >> dummyDescr;       myStream >> motionOffset;
        myStream >> dummyDescr;       myStream >> dummyValue;

        myFile.close();

}


void Animation::printCheatIndexTXT()
{
    qDebug() << "Animation::printCheatIndexTXT - totalAllignedFrames - " << totalAllignedFrames;
    qDebug() << "Animation::printCheatIndexTXT - motionOffset        - " << motionOffset;
    qDebug() << "\n";
}


