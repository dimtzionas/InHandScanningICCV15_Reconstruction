// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <modelSET.h>


ModelSet::ModelSet(){}
ModelSet::ModelSet(
                        QString INPUT_BasePath_OutPut_IN,
                        QString INPUT_dynamicStringPart_IN,
                        QString INPUT_EXTENSSS_Mesh_IN,
                        QString INPUT_EXTENSSS_Skeleton_IN,
                        QString INPUT_EXTENSSS_Skin_IN,
                        QString INPUT_PATH_MODELS_INFO,
                        bool    printEnabled
                  )
{

        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////
        read_MODELS_INFO_TXT( INPUT_PATH_MODELS_INFO );
        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////
        models.resize( INPUT_ModelNamesQVect.size() );
        models.squeeze();
        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////
        totalModels = models.size();
        ///////////////////////////////////////////////////
        ///////////////////////////////////////////////////

        for (int modelID=0; modelID<totalModels; modelID++)
        {
                models[modelID] = Model(      INPUT_BasePath_OutPut_IN,
                                              INPUT_dynamicStringPart_IN,
                                              INPUT_ModelNamesQVect[modelID],
                                              INPUT_EXTENSSS_Mesh_IN,
                                              INPUT_EXTENSSS_Skeleton_IN,
                                              INPUT_EXTENSSS_Skin_IN
                                       );
        }

        ///////////////////////////////////
        ///////////////////////////////////
        if (printEnabled)
        {
            print_MODELS_INFO_TXT();
            print_MODELS_INFO_analytic();
        }
        ///////////////////////////////////
        ///////////////////////////////////

}





void ModelSet::read_MODELS_INFO_TXT( QString myFileString_MODELS_INFO_TXT )
{

        QFile myFile( myFileString_MODELS_INFO_TXT );

        myFile.open(QIODevice::ReadOnly);

        if (myFile.exists() == false)   {   qDebug() << "MainWindow::read_MODELS_INFO_TXT - File **" << myFileString_MODELS_INFO_TXT << "** DOESN'T exist !!!!!";   return;     }
        if( !myFile.isOpen() )          {   qDebug() << "MainWindow::read_MODELS_INFO_TXT - File **" << myFileString_MODELS_INFO_TXT << "** DOESN'T open !!!!!";    return;     }

        QTextStream myTextStream(&myFile);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        int                           modelsNumber;
        myTextStream >>               modelsNumber;
        INPUT_ModelNamesQVect.resize( modelsNumber );

        for (int i=0; i<INPUT_ModelNamesQVect.size(); i++)     myTextStream >> INPUT_ModelNamesQVect[i];

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        myFile.close();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}





void ModelSet::print_MODELS_INFO_TXT()
{
                                                                qDebug() << "\n";
                                                                qDebug() << "ModelSet::print_MODELS_INFO_TXT \t" << INPUT_ModelNamesQVect.size();
    for (int mmm=0; mmm<INPUT_ModelNamesQVect.size(); mmm++)    qDebug() << "ModelSet::print_MODELS_INFO_TXT \t" << INPUT_ModelNamesQVect[mmm];
                                                                qDebug() << "\n";

}


void ModelSet::print_MODELS_INFO_analytic()
{

        for (int modelID=0; modelID<totalModels; modelID++)
        {
            qDebug() << "#################################################################################" << models[modelID].name << "\t" << modelID+1 << "/" << totalModels;
            qDebug() << "ModelSet::print_MODELS_INFO_analytic - totalBones            " << models[modelID].totalBones;
            qDebug() << "ModelSet::print_MODELS_INFO_analytic - totalSkeletonFrames   " << models[modelID].totalSkeletonFrames;
            qDebug() << "ModelSet::print_MODELS_INFO_analytic - totalSkinningBones    " << models[modelID].totalSkinningBones;
        }
            qDebug() << "\n";

}
