// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <touch.h>



////////////////////////////////////////////////////////
// typedef pcl::PointXYZRGB        TYPE_Point_Sensor; //
// typedef pcl::PointXYZRGBNormal  TYPE_Point_PostPr; //
////////////////////////////////////////////////////////



int Touch::detector_SkinTouch( int animNumb, pcl::search::KdTree<pcl::PointXYZRGBNormal> &tree, bool should_ONLY_EF, bool shouldPrint )
{


                    ///////////////////////////////////////////////////////////////
                    Animation *pAnimation = &(sequence->posedAnimations[animNumb]);
                    ///////////////////////////////////////////////////////////////


                    /////////////////////////////////////
                    pAnimation->touchingVertices.clear();
                    /////////////////////////////////////


                    /////////////////////////////////////////////////////
                    PARAM_SKIN_Dist_THRESH_CURR = PARAM_SKIN_Dist_THRESH;
                    /////////////////////////////////////////////////////


                    ///////////////////////////
                    int totalTouchingFINGs = 0;
                    ///////////////////////////

                    ////////////////////////
                    bool isFirstIter = true;
                    ////////////////////////


                    do
                    {


                            //////////////////////////////////////////////////////////////////////////////////////
                            //////////////////////////////////////////////////////////////////////////////////////
                            if (isFirstIter==false)    PARAM_SKIN_Dist_THRESH_CURR += PARAM_SKIN_Dist_THRESH_INCR;
                            //////////////////////////////////////////////////////////////////////////////////////
                            //////////////////////////////////////////////////////////////////////////////////////
                                isFirstIter = false;
                                ////////////////////
                                ////////////////////


                            for (int modelID=0; modelID<pAnimation->modelSet.totalModels; modelID++)
                            {

                                    ////////////////////////////////////////////////////////
                                    ////////////////////////////////////////////////////////
                                    Model *pModel = &(pAnimation->modelSet.models[modelID]);
                                    ////////////////////////////////////////////////////////
                                    ////////////////////////////////////////////////////////

                                    /////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////
                                    if (pModel->skinningSkeleton.skinningBones.size() == 1)     continue;
                                    /////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////


                                    for (int vvv=0; vvv<pModel->totalVertices; vvv++)
                                    {

                                                /////////////////////////////////////////////////////////////////////////////////
                                                /////////////////////////////////////////////////////////////////////////////////
                                                if (should_ONLY_EF && pModel->vertexBelongsToEndEffector(vvv)==false)   continue; // probably no gain with this !!!
                                                /////////////////////////////////////////////////////////////////////////////////
                                                /////////////////////////////////////////////////////////////////////////////////




                                                if (PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger)
                                                {

                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        int      _dominantSkinningBoneID   = pModel->find_Dominant_SkinningBone( vvv );
                                                        QString  _dominantSkinningBoneName = pModel->skinningSkeleton.skinningBones[ _dominantSkinningBoneID ].name;
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        int                                                         modelID, fingID, globalFingID;
                                                        skinTouch_fingID_2_globalFingID( _dominantSkinningBoneName, modelID, fingID, globalFingID );
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        if ( skinTouch_isFingerInTouch(                                              globalFingID ) )      continue;
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                                }




                                                /////////////////////////////////////////////////////////////////////////////////////////
                                                /////////////////////////////////////////////////////////////////////////////////////////
                                                double unsignedDIST = getDistance_SKIN_to_PCL_from_sensor( tree, (*(pModel->PCL))[vvv] );
                                                /////////////////////////////////////////////////////////////////////////////////////////
                                                /////////////////////////////////////////////////////////////////////////////////////////




                                                if ( unsignedDIST <= PARAM_SKIN_Dist_THRESH_CURR )
                                                {

                                                        TouchingVertex TMP_touchingVertex;
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.x = (float)pModel->mesh.verticesWeighted[vvv](0);
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.y = (float)pModel->mesh.verticesWeighted[vvv](1);
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.z = (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                       TMP_touchingVertex.vertex_Eigen_Vec3f  //=        pModel->mesh.verticesWeighted[vvv].cast<float>();
                                                                                                               << (float)pModel->mesh.verticesWeighted[vvv](0),
                                                                                                                  (float)pModel->mesh.verticesWeighted[vvv](1),
                                                                                                                  (float)pModel->mesh.verticesWeighted[vvv](2);
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.r = 0;
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.g = 255;
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.b = 255;
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.normal_x = pModel->mesh.normals_Vertices[vvv](0);
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.normal_y = pModel->mesh.normals_Vertices[vvv](1);
                                                                       TMP_touchingVertex.vertex_XYZRGBNormal.normal_z = pModel->mesh.normals_Vertices[vvv](2);
                                                                       TMP_touchingVertex.addressInVertices            = vvv;
                                                                       TMP_touchingVertex.dominantSkinningBoneID       = pModel->find_Dominant_SkinningBone( vvv );
                                                                       TMP_touchingVertex.dominantSkinningBoneName     = pModel->skinningSkeleton.skinningBones[ TMP_touchingVertex.dominantSkinningBoneID ].name;
                                                                       TMP_touchingVertex.distanceToPrimitive_Signed   = unsignedDIST;
                                                                       TMP_touchingVertex.modelID                      = modelID;
                                                                                                                                              /////////
                                                                   if (TMP_touchingVertex.dominantSkinningBoneName.contains("Lower Arm"))     continue;
                                                                                                                                              /////////
                                                        //////////////////////////////////////////////////////////
                                                        pAnimation->touchingVertices.append( TMP_touchingVertex );
                                                        //////////////////////////////////////////////////////////


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


                                                        if (shouldPrint)
                                                        {
                                                            std::cout << "Touch::detector_SkinTouch \t\t" << vvv << "\t" << modelID << "\t" << TMP_touchingVertex.vertex_XYZRGBNormal.x
                                                                                                                                    << "\t" << TMP_touchingVertex.vertex_XYZRGBNormal.y
                                                                                                                                    << "\t" << TMP_touchingVertex.vertex_XYZRGBNormal.z << "\t\t" << unsignedDIST << std::endl;
                                                        }

                                                }


                                                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                    } // vvv

                            } // modelID


                            if (shouldPrint)    std::cout << std::endl << std::endl << std::endl << std::endl;


                            /////////////////////////////////////////////////////////////////////
                            /////////////////////////////////////////////////////////////////////
                            totalTouchingFINGs = skinTouch_countDiffFingers_PER_ANIM( animNumb );
                            /////////////////////////////////////////////////////////////////////
                            /////////////////////////////////////////////////////////////////////


                    }
                    while (totalTouchingFINGs < PARAM_SKIN_minTouchingFINGs  /*&&  full_EF == false*/);


                    ///////////////////////////
                    return totalTouchingFINGs;
                    ///////////////////////////

}



