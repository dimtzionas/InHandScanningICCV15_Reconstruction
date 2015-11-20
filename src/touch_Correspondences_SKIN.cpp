// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <touch.h>

#include <pcl/registration/transforms.h>




void Touch::correspondencesFinder_SKIN_intersection()
{

    Animation* pAnimSource = &(sequence->posedAnimations[0]);  /*animNumb = 0 --> Source*/
    Animation* pAnimTarget = &(sequence->posedAnimations[1]);  /*animNumb = 1 --> Target*/


    ///////////////////////////
    myTouchCorrespSKIN.clear();
    ///////////////////////////


    bool shouldBreakInner = false;

    for     (int ttt_Source=0; ttt_Source<pAnimSource->touchingVertices.size(); ttt_Source++){    TouchingVertex* pTVertexSource = &(pAnimSource->touchingVertices[ttt_Source]);
        for (int ttt_Target=0; ttt_Target<pAnimTarget->touchingVertices.size(); ttt_Target++){    TouchingVertex* pTVertexTarget = &(pAnimTarget->touchingVertices[ttt_Target]);

            if (shouldBreakInner==true)
            {   shouldBreakInner = false;
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////
            int vvvSource = pTVertexSource->addressInVertices;      int mmmSource = pTVertexSource->modelID;
            int vvvTarget = pTVertexTarget->addressInVertices;      int mmmTarget = pTVertexTarget->modelID;
            ////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////


            if (mmmSource==mmmTarget  &&  vvvSource==vvvTarget)
            {
                         TouchCorresp_SKIN myTouchCorrespSKIN_TMP;
                                           myTouchCorrespSKIN_TMP.source = *pTVertexSource;
                                           myTouchCorrespSKIN_TMP.target = *pTVertexTarget;
                myTouchCorrespSKIN.append( myTouchCorrespSKIN_TMP );


                if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                {
                        //////////////////////////////////////////////////////////////////////////////////
                        //////////////////////////////////////////////////////////////////////////////////
                        Model* pModel_Source = &(sequence->posedAnimations[0].modelSet.models[mmmSource]);
                        Model* pModel_Target = &(sequence->posedAnimations[1].modelSet.models[mmmTarget]);
                        //////////////////////////////////////////////////////////////////////////////////
                        //////////////////////////////////////////////////////////////////////////////////
                        (*(pModel_Source->PCL))[vvvSource].r = 255; // yellow
                        (*(pModel_Source->PCL))[vvvSource].g = 255;
                        (*(pModel_Source->PCL))[vvvSource].b = 0;
                        ///////////////////////////////////////////
                        ///////////////////////////////////////////
                        (*(pModel_Target->PCL))[vvvTarget].r = 255; // yellow
                        (*(pModel_Target->PCL))[vvvTarget].g = 255;
                        (*(pModel_Target->PCL))[vvvTarget].b = 0;
                        ///////////////////////////////////////////
                        ///////////////////////////////////////////
                }

                shouldBreakInner = true;
            }

        }

    }

}



void Touch::TouchCorrespSKIN_TRASNFORM_Source( Eigen::Matrix4f transformationMatrix4f )
{

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr            myTouchCorrespSKIN_SOURCE( new pcl::PointCloud<pcl::PointXYZRGBNormal> );

        for (int ttt=0; ttt<myTouchCorrespSKIN.size(); ttt++)   myTouchCorrespSKIN_SOURCE->push_back( myTouchCorrespSKIN[ttt].source.vertex_XYZRGBNormal );

                          //////////////////////////////////////////////////////////////////////////////////////////
                          pcl::transformPointCloudWithNormals( *myTouchCorrespSKIN_SOURCE, /////////////////////////
                                                               *myTouchCorrespSKIN_SOURCE, transformationMatrix4f );
                          //////////////////////////////////////////////////////////////////////////////////////////

        for (int ttt=0; ttt<myTouchCorrespSKIN.size(); ttt++)
        {
                myTouchCorrespSKIN[ttt].source.vertex_XYZRGBNormal = (*myTouchCorrespSKIN_SOURCE)[ttt];
                myTouchCorrespSKIN[ttt].source.vertex_Eigen_Vec3f <<   myTouchCorrespSKIN[ttt].source.vertex_XYZRGBNormal.x,
                                                                       myTouchCorrespSKIN[ttt].source.vertex_XYZRGBNormal.y,
                                                                       myTouchCorrespSKIN[ttt].source.vertex_XYZRGBNormal.z;
        }

}



void Touch::TouchCorrespSKIN_TRASNFORM_Target( Eigen::Matrix4f transformationMatrix4f )
{

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr            myTouchCorrespSKIN_TARGET( new pcl::PointCloud<pcl::PointXYZRGBNormal> );

        for (int ttt=0; ttt<myTouchCorrespSKIN.size(); ttt++)   myTouchCorrespSKIN_TARGET->push_back( myTouchCorrespSKIN[ttt].target.vertex_XYZRGBNormal );

                          //////////////////////////////////////////////////////////////////////////////////////////
                          pcl::transformPointCloudWithNormals( *myTouchCorrespSKIN_TARGET, /////////////////////////
                                                               *myTouchCorrespSKIN_TARGET, transformationMatrix4f );
                          //////////////////////////////////////////////////////////////////////////////////////////

        for (int ttt=0; ttt<myTouchCorrespSKIN.size(); ttt++)
        {
                myTouchCorrespSKIN[ttt].target.vertex_XYZRGBNormal = (*myTouchCorrespSKIN_TARGET)[ttt];
                myTouchCorrespSKIN[ttt].target.vertex_Eigen_Vec3f <<   myTouchCorrespSKIN[ttt].target.vertex_XYZRGBNormal.x,
                                                                       myTouchCorrespSKIN[ttt].target.vertex_XYZRGBNormal.y,
                                                                       myTouchCorrespSKIN[ttt].target.vertex_XYZRGBNormal.z;
        }

}





void Touch::correspondencesPrinter_SKIN()
{
    for (int ttt=0; ttt<myTouchCorrespSKIN.size(); ttt++)
    {
            std::cout << "Touch::correspondencesPrinter_SKIN \t source \t" << myTouchCorrespSKIN[ttt].source.addressInVertices                      << "\t"
                                                                           << myTouchCorrespSKIN[ttt].source.dominantSkinningBoneID                 << "\t"
                                                                           << myTouchCorrespSKIN[ttt].source.dominantSkinningBoneName.toStdString() << "\t"
                                                                           << myTouchCorrespSKIN[ttt].source.distanceToPrimitive_Signed             << "\t"

                                                            "\t target \t" << myTouchCorrespSKIN[ttt].target.addressInVertices                      << "\t"
                                                                           << myTouchCorrespSKIN[ttt].target.dominantSkinningBoneID                 << "\t"
                                                                           << myTouchCorrespSKIN[ttt].target.dominantSkinningBoneName.toStdString() << "\t"
                                                                           << myTouchCorrespSKIN[ttt].target.distanceToPrimitive_Signed             << "\t";
                                                                 std::cout << std::endl;
    }
    if (myTouchCorrespSKIN.size() == 0)
    {
        std::cout << "Touch::mymyTouchCorrespSKIN_print \t NO CORRESPONDENCE" << std::endl << std::endl << std::endl;
    }
    std::cout << std::endl;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Touch::correspondencesFinder_SKIN_full()
{

        Animation* pAnimSource = &(sequence->posedAnimations[0]);  /*animNumb = 0 --> Source*/
        Animation* pAnimTarget = &(sequence->posedAnimations[1]);  /*animNumb = 1 --> Target*/


        ///////////////////////////
        myTouchCorrespSKIN.clear();
        ///////////////////////////


        int SIZ  = pAnimSource->touchingVertices.size();
        int SIZ2 = pAnimTarget->touchingVertices.size();


        if (SIZ != SIZ2)   {   std::cout << "\n\n" << "correspondencesFinder_SKIN_full - DIMENSIONS PROBLEM - SIZ !!!" << "\n\n" << std::endl;   exit(1);   }


        for (int ttt=0; ttt<pAnimSource->touchingVertices.size(); ttt++)
        {


                    TouchingVertex* pTVertexSource = &(pAnimSource->touchingVertices[ttt]);
                    TouchingVertex* pTVertexTarget = &(pAnimTarget->touchingVertices[ttt]);


                    ////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////////////
                    int vvvSource = pTVertexSource->addressInVertices;      int mmmSource = pTVertexSource->modelID;
                    int vvvTarget = pTVertexTarget->addressInVertices;      int mmmTarget = pTVertexTarget->modelID;
                    ////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////////////


                    if ( vvvSource != vvvTarget || mmmSource != mmmTarget  )   {   std::cout << "\n\n" << "correspondencesFinder_SKIN_full - DIMENSIONS PROBLEM - vvv/mmm !!!" << "\n\n" << std::endl;   exit(1);   }


                    TouchCorresp_SKIN          myTouchCorrespSKIN_TMP;
                                               myTouchCorrespSKIN_TMP.source = *pTVertexSource;
                                               myTouchCorrespSKIN_TMP.target = *pTVertexTarget;
                    myTouchCorrespSKIN.append( myTouchCorrespSKIN_TMP );


                    if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                    {
                            //////////////////////////////////////////////////////////////////////////////////
                            //////////////////////////////////////////////////////////////////////////////////
                            Model* pModel_Source = &(sequence->posedAnimations[0].modelSet.models[mmmSource]);
                            Model* pModel_Target = &(sequence->posedAnimations[1].modelSet.models[mmmTarget]);
                            //////////////////////////////////////////////////////////////////////////////////
                            //////////////////////////////////////////////////////////////////////////////////
                            (*(pModel_Source->PCL))[vvvSource].r = 255; // yellow
                            (*(pModel_Source->PCL))[vvvSource].g = 255;
                            (*(pModel_Source->PCL))[vvvSource].b = 0;
                            ///////////////////////////////////////////
                            ///////////////////////////////////////////
                            (*(pModel_Target->PCL))[vvvTarget].r = 255; // yellow
                            (*(pModel_Target->PCL))[vvvTarget].g = 255;
                            (*(pModel_Target->PCL))[vvvTarget].b = 0;
                            ///////////////////////////////////////////
                            ///////////////////////////////////////////
                    }

        }

}



struct touch_DominantSkinningBoneSTRUCT
{
    int             modelID;
    int              fingID;
    int      skinningBoneID;
    QString  skinningBoneName;
    double   distance;
};



void Touch::detector_SkinTouch_ExpandTouchingBones_at_FULL_BONE()
{

        Animation* pAnimSource = &(sequence->posedAnimations[0]);  /*animNumb = 0 --> Source*/
        Animation* pAnimTarget = &(sequence->posedAnimations[1]);  /*animNumb = 1 --> Target*/


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////
        QVector<touch_DominantSkinningBoneSTRUCT>  touch_DominantSkinningBoneVECT;
                                                   touch_DominantSkinningBoneVECT.clear();
        //////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////


        /////////////////////////////////////////
        skinTouch_countDiffFingers_PER_ANIM( 0 );   // updates **touchingFinger_COUNTer**
        /////////////////////////////////////////

        for (int iii=0; iii<touchingFinger_COUNTer.size(); iii++)
        {

                 if (touchingFinger_COUNTer[iii].counter > 0)
                 {

                     touch_DominantSkinningBoneSTRUCT       touch_DominantSkinningBoneTMP;
                                                            touch_DominantSkinningBoneTMP.       modelID   = touchingFinger_COUNTer[iii].modelID;
                                                            touch_DominantSkinningBoneTMP.        fingID   = touchingFinger_COUNTer[iii].fingID;
                                                            touch_DominantSkinningBoneTMP.skinningBoneID   = touchingFinger_COUNTer[iii].skinningBoneID;
                                                            touch_DominantSkinningBoneTMP.skinningBoneName = touchingFinger_COUNTer[iii].name_skinningBone;
                                                            touch_DominantSkinningBoneTMP.distance         = touchingFinger_COUNTer[iii].signedDist;
                     touch_DominantSkinningBoneVECT.append( touch_DominantSkinningBoneTMP );
                 }

        }

        /////////////////////////////////////////
        skinTouch_countDiffFingers_PER_ANIM( 1 );   // updates **touchingFinger_COUNTer**
        /////////////////////////////////////////

        for (int iii=0; iii<touchingFinger_COUNTer.size(); iii++)
        {

                 if (touchingFinger_COUNTer[iii].counter > 0)
                 {
                                                                                                                                       ///////////////////////////////
                                                                                                                                       bool shouldContinue = false;
                     for (int jjj=0; jjj<touchingFinger_COUNTer.size(); jjj++)  if (touchingFinger_COUNTer[jjj].name_skinningBone ==
                                                                                    touchingFinger_COUNTer[iii].name_skinningBone  )        shouldContinue = true;
                                                                                                                                        if (shouldContinue)  continue;   // Avoids Dublicate Bones !!!
                                                                                                                                       ///////////////////////////////

                     touch_DominantSkinningBoneSTRUCT       touch_DominantSkinningBoneTMP;
                                                            touch_DominantSkinningBoneTMP.       modelID   = touchingFinger_COUNTer[iii].modelID;
                                                            touch_DominantSkinningBoneTMP.        fingID   = touchingFinger_COUNTer[iii].fingID;
                                                            touch_DominantSkinningBoneTMP.skinningBoneID   = touchingFinger_COUNTer[iii].skinningBoneID;
                                                            touch_DominantSkinningBoneTMP.skinningBoneName = touchingFinger_COUNTer[iii].name_skinningBone;
                                                            touch_DominantSkinningBoneTMP.distance         = touchingFinger_COUNTer[iii].signedDist;
                     touch_DominantSkinningBoneVECT.append( touch_DominantSkinningBoneTMP );
                 }

        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////
        //////////////////////////////////////
        pAnimSource->touchingVertices.clear();
        pAnimTarget->touchingVertices.clear();
        //////////////////////////////////////
        //////////////////////////////////////


        for (int iii=0; iii<touch_DominantSkinningBoneVECT.size(); iii++)
        {

                        int     modelID = touch_DominantSkinningBoneVECT[iii].modelID;
                        int        skkk = touch_DominantSkinningBoneVECT[iii].skinningBoneID;
                        double distance = touch_DominantSkinningBoneVECT[iii].distance;


                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                        /////////////////////////////////////////
                        /////////////////////////////////////////
                        ////  pAnimSource->touchingVertices  ////
                        /////////////////////////////////////////
                        /////////////////////////////////////////


                        for (int jjj=0;    jjj<pAnimSource->modelSet.models[modelID].skinningSkeleton.skinningBones[skkk].mainlyInfluenced_VertexIDs.size(); jjj++)
                        {    int           vvv=pAnimSource->modelSet.models[modelID].skinningSkeleton.skinningBones[skkk].mainlyInfluenced_VertexIDs[jjj];

                             Model* pModel = &(pAnimSource->modelSet.models[modelID]);

                                     TouchingVertex                        TMP_touchingVertex;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.x = (float)pModel->mesh.verticesWeighted[vvv](0);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.y = (float)pModel->mesh.verticesWeighted[vvv](1);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.z = (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                           TMP_touchingVertex.vertex_Eigen_Vec3f   << (float)pModel->mesh.verticesWeighted[vvv](0),
                                                                                                                      (float)pModel->mesh.verticesWeighted[vvv](1),
                                                                                                                      (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.r = 255;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.g = 255;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.b = 0;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_x = pModel->mesh.normals_Vertices[vvv](0);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_y = pModel->mesh.normals_Vertices[vvv](1);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_z = pModel->mesh.normals_Vertices[vvv](2);
                                                                           TMP_touchingVertex.addressInVertices            = vvv;
                                                                           TMP_touchingVertex.dominantSkinningBoneID       = skkk;
                                                                           TMP_touchingVertex.dominantSkinningBoneName     = pModel->skinningSkeleton.skinningBones[ TMP_touchingVertex.dominantSkinningBoneID ].name;
                                                                           TMP_touchingVertex.distanceToPrimitive_Signed   = distance;
                                                                           TMP_touchingVertex.modelID                      = modelID;
                                     ///////////////////////////////////////////////////////////
                                     pAnimSource->touchingVertices.append( TMP_touchingVertex );
                                     ///////////////////////////////////////////////////////////


                                     if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                                     {
                                             //////////////////////////////
                                             //////////////////////////////
                                             (*(pModel->PCL))[vvv].r = 0;    // cyan
                                             (*(pModel->PCL))[vvv].g = 255;
                                             (*(pModel->PCL))[vvv].b = 255;
                                             //////////////////////////////
                                             //////////////////////////////
                                     }


                        } // for (int jjj=0;


                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                        /////////////////////////////////////////
                        /////////////////////////////////////////
                        ////  pAnimTarget->touchingVertices  ////
                        /////////////////////////////////////////
                        /////////////////////////////////////////


                        for (int jjj=0;    jjj<pAnimTarget->modelSet.models[modelID].skinningSkeleton.skinningBones[skkk].mainlyInfluenced_VertexIDs.size(); jjj++)
                        {    int           vvv=pAnimTarget->modelSet.models[modelID].skinningSkeleton.skinningBones[skkk].mainlyInfluenced_VertexIDs[jjj];

                             Model* pModel = &(pAnimTarget->modelSet.models[modelID]);

                                     TouchingVertex                        TMP_touchingVertex;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.x = (float)pModel->mesh.verticesWeighted[vvv](0);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.y = (float)pModel->mesh.verticesWeighted[vvv](1);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.z = (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                           TMP_touchingVertex.vertex_Eigen_Vec3f   << (float)pModel->mesh.verticesWeighted[vvv](0),
                                                                                                                      (float)pModel->mesh.verticesWeighted[vvv](1),
                                                                                                                      (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.r = 255;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.g = 255;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.b = 0;
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_x = pModel->mesh.normals_Vertices[vvv](0);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_y = pModel->mesh.normals_Vertices[vvv](1);
                                                                           TMP_touchingVertex.vertex_XYZRGBNormal.normal_z = pModel->mesh.normals_Vertices[vvv](2);
                                                                           TMP_touchingVertex.addressInVertices            = vvv;
                                                                           TMP_touchingVertex.dominantSkinningBoneID       = skkk;
                                                                           TMP_touchingVertex.dominantSkinningBoneName     = pModel->skinningSkeleton.skinningBones[ TMP_touchingVertex.dominantSkinningBoneID ].name;
                                                                           TMP_touchingVertex.distanceToPrimitive_Signed   = distance;
                                                                           TMP_touchingVertex.modelID                      = modelID;
                                     ///////////////////////////////////////////////////////////
                                     pAnimTarget->touchingVertices.append( TMP_touchingVertex );
                                     ///////////////////////////////////////////////////////////


                                     if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                                     {
                                             //////////////////////////////
                                             //////////////////////////////
                                             (*(pModel->PCL))[vvv].r = 0;    // cyan
                                             (*(pModel->PCL))[vvv].g = 255;
                                             (*(pModel->PCL))[vvv].b = 255;
                                             //////////////////////////////
                                             //////////////////////////////
                                     }


                        } // for (int jjj=0;

                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        } // for (int iii=0; iii<touch_DominantSkinningBoneVECT.size(); iii++)


}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Touch::correspondencesFinder_SKIN_union()
{

    Animation* pAnimSource = &(sequence->posedAnimations[0]);  /*animNumb = 0 --> Source*/
    Animation* pAnimTarget = &(sequence->posedAnimations[1]);  /*animNumb = 1 --> Target*/

    correspondencesFinder_SKIN_union( pAnimSource->touchingVertices, pAnimSource->modelSet,
                                      pAnimTarget->touchingVertices, pAnimTarget->modelSet);

}


void Touch::correspondencesFinder_SKIN_union( QVector<TouchingVertex>& touchingVertices_Source, ModelSet& modelSet_Source,
                                              QVector<TouchingVertex>& touchingVertices_Target, ModelSet& modelSet_Target )
{

        ///////////////////////////
        myTouchCorrespSKIN.clear();
        ///////////////////////////


        for (int tvv=0; tvv<touchingVertices_Source.size(); tvv++)
        {

                TouchingVertex  pTVertexSource = touchingVertices_Source[tvv];

                TouchingVertex   TVertexTarget_TMP;
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.x =        modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](0);
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.y =        modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](1);
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.z =        modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](2);
                                 TVertexTarget_TMP.vertex_Eigen_Vec3f   << (float)modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](0),
                                                                           (float)modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](1),
                                                                           (float)modelSet_Target.models[pTVertexSource.modelID].mesh.verticesWeighted[pTVertexSource.addressInVertices](2);
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.r = 255;
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.g = 255;
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.b = 0;
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.normal_x = modelSet_Target.models[pTVertexSource.modelID].mesh.normals_Vertices[pTVertexSource.addressInVertices](0);
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.normal_y = modelSet_Target.models[pTVertexSource.modelID].mesh.normals_Vertices[pTVertexSource.addressInVertices](1);
                                 TVertexTarget_TMP.vertex_XYZRGBNormal.normal_z = modelSet_Target.models[pTVertexSource.modelID].mesh.normals_Vertices[pTVertexSource.addressInVertices](2);
                                 TVertexTarget_TMP.addressInVertices            = pTVertexSource.addressInVertices;
                                 TVertexTarget_TMP.dominantSkinningBoneID       = modelSet_Target.models[pTVertexSource.modelID].find_Dominant_SkinningBone(    pTVertexSource.addressInVertices );
                                 TVertexTarget_TMP.dominantSkinningBoneName     = modelSet_Target.models[pTVertexSource.modelID].skinningSkeleton.skinningBones[ TVertexTarget_TMP.dominantSkinningBoneID ].name;
                                 TVertexTarget_TMP.distanceToPrimitive_Signed   = -666666;
                                 TVertexTarget_TMP.modelID                      = pTVertexSource.modelID;

                ///////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////
                         TouchCorresp_SKIN myTouchCorrespSKIN_TMP;
                                           myTouchCorrespSKIN_TMP.source = pTVertexSource;
                                           myTouchCorrespSKIN_TMP.target =  TVertexTarget_TMP;
                myTouchCorrespSKIN.append( myTouchCorrespSKIN_TMP );
                ///////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////

                if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                {
                        Model* pModel_Source = &(sequence->posedAnimations[0].modelSet.models[pTVertexSource.    modelID]);
                        Model* pModel_Target = &(sequence->posedAnimations[1].modelSet.models[ TVertexTarget_TMP.modelID]);
                        int vvvSource = pTVertexSource.   addressInVertices;
                        int vvvTarget =  TVertexTarget_TMP.addressInVertices;

                        (*(pModel_Source->PCL))[vvvSource].r = 255;
                        (*(pModel_Source->PCL))[vvvSource].g = 255;
                        (*(pModel_Source->PCL))[vvvSource].b = 0;

                        (*(pModel_Target->PCL))[vvvTarget].r = 255;
                        (*(pModel_Target->PCL))[vvvTarget].g = 255;
                        (*(pModel_Target->PCL))[vvvTarget].b = 0;
                }

                ///////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////

        }


        for (int tvv=0; tvv<touchingVertices_Target.size(); tvv++)
        {

                //////////////////////////
                bool alreadyThere = false;
                //////////////////////////


                TouchingVertex  pTVertexTarget = touchingVertices_Target[tvv];


                for (int ccc=0; ccc<myTouchCorrespSKIN.size(); ccc++)
                {
                        TouchingVertex* pTVertexTarget_ALREADY = &(myTouchCorrespSKIN[ccc].target);

                        if (pTVertexTarget.modelID==pTVertexTarget_ALREADY->modelID  &&  pTVertexTarget.addressInVertices==pTVertexTarget_ALREADY->addressInVertices)
                        {
                            alreadyThere = true;
                            break;
                        }
                }


                if (alreadyThere==false)
                {
                        TouchingVertex   TVertexSource_TMP;
                                         TVertexSource_TMP.vertex_XYZRGBNormal.x =        modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](0);
                                         TVertexSource_TMP.vertex_XYZRGBNormal.y =        modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](1);
                                         TVertexSource_TMP.vertex_XYZRGBNormal.z =        modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](2);
                                         TVertexSource_TMP.vertex_Eigen_Vec3f   << (float)modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](0),
                                                                                   (float)modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](1),
                                                                                   (float)modelSet_Source.models[pTVertexTarget.modelID].mesh.verticesWeighted[pTVertexTarget.addressInVertices](2);
                                         TVertexSource_TMP.vertex_XYZRGBNormal.r = 255;
                                         TVertexSource_TMP.vertex_XYZRGBNormal.g = 255;
                                         TVertexSource_TMP.vertex_XYZRGBNormal.b = 0;
                                         TVertexSource_TMP.vertex_XYZRGBNormal.normal_x = modelSet_Source.models[pTVertexTarget.modelID].mesh.normals_Vertices[pTVertexTarget.addressInVertices](0);
                                         TVertexSource_TMP.vertex_XYZRGBNormal.normal_y = modelSet_Source.models[pTVertexTarget.modelID].mesh.normals_Vertices[pTVertexTarget.addressInVertices](1);
                                         TVertexSource_TMP.vertex_XYZRGBNormal.normal_z = modelSet_Source.models[pTVertexTarget.modelID].mesh.normals_Vertices[pTVertexTarget.addressInVertices](2);
                                         TVertexSource_TMP.addressInVertices            = pTVertexTarget.addressInVertices;
                                         TVertexSource_TMP.dominantSkinningBoneID       = modelSet_Source.models[pTVertexTarget.modelID].find_Dominant_SkinningBone(    pTVertexTarget.addressInVertices );
                                         TVertexSource_TMP.dominantSkinningBoneName     = modelSet_Source.models[pTVertexTarget.modelID].skinningSkeleton.skinningBones[ TVertexSource_TMP.dominantSkinningBoneID ].name;
                                         TVertexSource_TMP.distanceToPrimitive_Signed   = -666666;
                                         TVertexSource_TMP.modelID                      = pTVertexTarget.modelID;

                        ///////////////////////////////////////////////////////////////////////////////
                        ///////////////////////////////////////////////////////////////////////////////
                                 TouchCorresp_SKIN myTouchCorrespSKIN_TMP;
                                                   myTouchCorrespSKIN_TMP.target = pTVertexTarget;
                                                   myTouchCorrespSKIN_TMP.source =  TVertexSource_TMP;
                        myTouchCorrespSKIN.append( myTouchCorrespSKIN_TMP );
                        ///////////////////////////////////////////////////////////////////////////////
                        ///////////////////////////////////////////////////////////////////////////////
                        Model* pModel_Source = &(sequence->posedAnimations[0].modelSet.models[ TVertexSource_TMP.modelID]);
                        Model* pModel_Target = &(sequence->posedAnimations[1].modelSet.models[pTVertexTarget.    modelID]);
                        int vvvSource =  TVertexSource_TMP.addressInVertices;
                        int vvvTarget = pTVertexTarget.    addressInVertices;
                        /////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////
                        if (PARAM_DBG_RenderMesh_Color_TouchCORR)
                        {
                                (*(pModel_Source->PCL))[vvvSource].r = 255;
                                (*(pModel_Source->PCL))[vvvSource].g = 255;
                                (*(pModel_Source->PCL))[vvvSource].b = 0;

                                (*(pModel_Target->PCL))[vvvTarget].r = 255;
                                (*(pModel_Target->PCL))[vvvTarget].g = 255;
                                (*(pModel_Target->PCL))[vvvTarget].b = 0;
                        }
                        /////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////
                }

        }

}


