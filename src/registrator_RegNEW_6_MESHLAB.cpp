// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>



void Registrator::myReg_X_MESHLAB_local( QString OUTPUT_PATH )
{

        std::cout << "\n\n\n" << "myReg_X_MESHLAB_local" << "\n\n" << std::endl;

        QString pathINN = OUTPUT_PATH + "___OUT___2___pairwiseAlignment_TSDF.ply";
        QString pathOUT = OUTPUT_PATH + "___OUT___3___FINAL_Watertight_MESH.ply";
        QString pathSCR = "registrator_RegNEW_6_MESLAB_SCRIPT.mlx";

        QDir                                qdir( qApp->applicationDirPath() );
                                            qdir.cdUp();
        if (IS_ROOT_DIR(qdir) == false)     qdir.cdUp();

        pathSCR =                           qdir.path() + "/src/" + pathSCR;

        std::cout <<                          std::endl;
        std::cout << pathINN.toStdString() << std::endl;
        std::cout << pathOUT.toStdString() << std::endl;
        std::cout << pathSCR.toStdString() << std::endl;
        std::cout <<                          std::endl;

        QString COMMAND_STR = "meshlabserver -i " + pathINN + " -o " + pathOUT + " -s " + pathSCR;

        std::cout << COMMAND_STR.toStdString() << std::endl;
        std::cout <<                              std::endl;

        int output = system( COMMAND_STR.toLocal8Bit() );
            output;

        std::cout << "\n\n\n" << "FINAL OUTPUT -> " << pathOUT.toStdString() << "\n\n\n" << std::endl;

}


