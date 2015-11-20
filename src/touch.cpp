// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "touch.h"


Touch::Touch(){}

Touch::Touch( Sequence* sequence )
{
    this->sequence = sequence;

    myTouchCorrespSKIN.    clear();
    touchingFinger_COUNTer.clear();
    touchingFinger_COUNTer.resize(10);
}


// animID 0 - source
// animID 1 - target

void Touch::TouchingVertices_TRASNFORM_Source(  Eigen::Matrix4f transformationMatrix4f )  {          sequence->posedAnimations[0].TouchingVertices_TRASNFORM( transformationMatrix4f );   }
void Touch::TouchingVertices_TRASNFORM_Target(  Eigen::Matrix4f transformationMatrix4f )  {          sequence->posedAnimations[1].TouchingVertices_TRASNFORM( transformationMatrix4f );   }

void Touch::TouchingVertices_PRINT_Source()                                               {          sequence->posedAnimations[0].TouchingVertices_PRINT();   }
void Touch::TouchingVertices_PRINT_Target()                                               {          sequence->posedAnimations[1].TouchingVertices_PRINT();   }

int  Touch::TouchingVertices_READ_Source(       QString FULL_PATH_READ )                  {   return sequence->posedAnimations[0].TouchingVertices_READ(  FULL_PATH_READ );   }
int  Touch::TouchingVertices_READ_Target(       QString FULL_PATH_READ )                  {   return sequence->posedAnimations[1].TouchingVertices_READ(  FULL_PATH_READ );   }

void Touch::TouchingVertices_WRITE_Source(      QString FULL_PATH_SAVE )                  {          sequence->posedAnimations[0].TouchingVertices_WRITE( FULL_PATH_SAVE );   }
void Touch::TouchingVertices_WRITE_Target(      QString FULL_PATH_SAVE )                  {          sequence->posedAnimations[1].TouchingVertices_WRITE( FULL_PATH_SAVE );   }


                                                                                                                                      // use whichever animID u like, it doesn't matter !!!!!
void Touch::TouchingVertices_PRINT_DBG(                            const QVector< TouchingVertex > &myTouchingVertices )   {          sequence->posedAnimations[0].TouchingVertices_PRINT_DBG(                 myTouchingVertices );   }
int  Touch::TouchingVertices_READ_DBG(   QString FULL_PATH_READ,         QVector< TouchingVertex > &myTouchingVertices )   {   return sequence->posedAnimations[0].TouchingVertices_READ_DBG(  FULL_PATH_READ, myTouchingVertices );   }
