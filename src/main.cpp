// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <QApplication>

#include <registrator.h>

int main(int argc, char *argv[])
{
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    QApplication app(argc, argv); // -> QCoreApplication::applicationDirPath
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if (argc < 4 + 1)
    {
        std::cout << "\n\n" << "There are missing arguments !!!" << "\n\n" << std::endl;
        exit(1);
    }
    if (argc > 4 + 1)
    {
        std::cout << "\n\n" << "There are redundant arguments !!!" << "\n\n" << std::endl;
        exit(1);
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    QVector<QString> inputArguments;
                     inputArguments.resize(argc);
    for  (int i=0; i<inputArguments.size(); i++)
                     inputArguments[i] = QString::fromLocal8Bit(argv[i]);
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if (inputArguments[1].endsWith(".txt") == false)
        inputArguments[1] +=       ".txt";
    if (inputArguments[2].endsWith(".txt") == false)
        inputArguments[2] +=       ".txt";
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    bool flag_3 = false;
    bool flag_4 = false;
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if      (inputArguments[3].toLower() == "false")         flag_3 = false;
    else if (inputArguments[3].toLower() == "true")          flag_3 = true;
    else    {
        std::cout << "\n\n" << "Argument No3 should be true/false !!!" << "\n\n" << std::endl;
        exit(1);
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if      (inputArguments[4].toLower() == "false")         flag_4 = false;
    else if (inputArguments[4].toLower() == "true")          flag_4 = true;
    else    {
        std::cout << "\n\n" << "Argument No4 should be true/false !!!" << "\n\n" << std::endl;
        exit(1);
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////
    /////////////////////////////////////////////
    Registrator registrator( inputArguments[1],
                             inputArguments[2],
                                       flag_3,
                                       flag_4);
    /////////////////////////////////////////////
    /////////////////////////////////////////////


}
