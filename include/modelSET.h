// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef MODELSET_H
#define MODELSET_H

#include <model.h>


class ModelSet
{
public:
    ModelSet();
    ModelSet(
                    QString INPUT_BasePath_OutPut_IN,
                    QString INPUT_dynamicStringPart_IN,
                    QString INPUT_EXTENSSS_Mesh_IN,
                    QString INPUT_EXTENSSS_Skeleton_IN,
                    QString INPUT_EXTENSSS_Skin_IN,
                    QString INPUT_PATH_MODELS_INFO,
                    bool    printEnabled
            );

    QVector< Model > models;

    int totalModels;

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    void  read_MODELS_INFO_TXT( QString myFileString_MODELS_INFO_TXT );
    void print_MODELS_INFO_TXT();
    void print_MODELS_INFO_analytic();

    QVector< QString > INPUT_ModelNamesQVect;
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

};

#endif // MODELSET_H
