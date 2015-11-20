// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "animation.h"



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Animation::transform_ModelSet_MESHES( int frameNumber_XwrisOffset )
{


                ////////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////////
                if (  (frameNumber_XwrisOffset < 0)  ||  (frameNumber_XwrisOffset >= totalAllignedFrames)  )
                {
                    qDebug() << "\n\n\n   Animation::transform_ModelSet_MESHES - OutOfBounds ERROR  \n\n\n";
                    return;
                }
                ////////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////////

                /////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////
                int frameNumber = frameNumber_XwrisOffset + motionOffset;
                /////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////

                //////////////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////

                Eigen::MatrixXd mat_0(         4,4);
                Eigen::MatrixXd mat_c(         4,4);
                Eigen::MatrixXd mat_C_0inv(    4,4);
                Eigen::MatrixXd vertex(        4,1);
                Eigen::MatrixXd weightedVertex(4,1);

                //////////////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////

                for (int modelID=0; modelID<modelSet.totalModels; modelID++)
                {
                            ////////////////////////////////////////////
                            Model *pModel = &(modelSet.models[modelID]);
                            ////////////////////////////////////////////

                            //////////////////////////////////////////////////////////////
                            pModel->mesh.verticesWeighted.fill( Eigen::Vector3d::Zero() );
                            //////////////////////////////////////////////////////////////

                            for (int skkk=0; skkk<pModel->totalSkinningBones; skkk++)
                            {

                                                            ///////////////////////////////////////////////////////////////////////////
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            int bbb = pModel->lookup_SkinningBone_to_Bones_ID[skkk];
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            mat_0 << movingModels[modelID].motionFrames[     0     ].bones[bbb].RT_4x4;
                                                            mat_c << movingModels[modelID].motionFrames[frameNumber].bones[bbb].RT_4x4;
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            mat_C_0inv = mat_c * mat_0.inverse();
                                                            ///////////////////////////////////////////////////////////////////////////
                                                            ///////////////////////////////////////////////////////////////////////////

                                                            for (int vvv=0; vvv<pModel->totalVertices; vvv++)
                                                            {

                                                                    double skinnWeight = pModel->skin.skinnedVertices[vvv].skinWeights[skkk];


                                                                    if  (skinnWeight > 0)
                                                                    {
                                                                            vertex << pModel->mesh.vertices[vvv](0),
                                                                                      pModel->mesh.vertices[vvv](1),
                                                                                      pModel->mesh.vertices[vvv](2),
                                                                                      1;

                                                                            weightedVertex = skinnWeight * mat_C_0inv * vertex;

                                                                            pModel->mesh.verticesWeighted[vvv](0) += weightedVertex(0);
                                                                            pModel->mesh.verticesWeighted[vvv](1) += weightedVertex(1);
                                                                            pModel->mesh.verticesWeighted[vvv](2) += weightedVertex(2);
                                                                    }

                                                            } // vvv

                            } // for for (int skkk=0; skkk<modelSet.models[modelID].totalSkinningBones; skkk++)


                            ////////////////////////////////////////////////////////////////////////////////
                            for (int vvv=0; vvv<pModel->totalVertices; vvv++)
                            {
                                pModel->mesh.verticesOLD[vvv] = pModel->mesh.verticesWeighted[vvv];
                            }
                            ////////////////////////////////////////////////////////////////////////////////


                            //////////////////////////
                            //////////////////////////
                            pModel->compute_NORMALS();
                            //////////////////////////
                            //////////////////////////

                } // for (int modelID=0; modelID<modelSet.totalModels; modelID++)

}





void Animation::transform_ModelSet_MESHES( const Eigen::Matrix4f& transf_4x4 )
{

                for (int modelID=0; modelID<modelSet.totalModels; modelID++)
                {

                            ////////////////////////////////////////////
                            Model *pModel = &(modelSet.models[modelID]);
                            ////////////////////////////////////////////

                            for (int vvv=0; vvv<pModel->totalVertices; vvv++)
                            {

                                    Eigen::MatrixXd weightedVertex(4,1);
                                                    weightedVertex << pModel->mesh.verticesWeighted[vvv](0),
                                                                      pModel->mesh.verticesWeighted[vvv](1),
                                                                      pModel->mesh.verticesWeighted[vvv](2),
                                                                      1;

                                                    weightedVertex = transf_4x4.cast<double>() * weightedVertex;

                                                    pModel->mesh.verticesWeighted[vvv] << weightedVertex(0),
                                                                                          weightedVertex(1),
                                                                                          weightedVertex(2);

                            } // vvv

                            ////////////////////////////////////////////////////////////////////////////////
                            for (int vvv=0; vvv<pModel->totalVertices; vvv++)
                            {
                                pModel->mesh.verticesOLD[vvv] = pModel->mesh.verticesWeighted[vvv];
                            }
                            ////////////////////////////////////////////////////////////////////////////////


                            //////////////////////////
                            //////////////////////////
                            pModel->compute_NORMALS();
                            //////////////////////////
                            //////////////////////////

                } // for (int modelID=0; modelID<modelSet.totalModels; modelID++)


}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void Animation::transform_ModelSet_SKELETONS( int frameNumber_XwrisOffset )
{

        ////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (  (frameNumber_XwrisOffset < 0)  ||  (frameNumber_XwrisOffset >= totalAllignedFrames)  )
        {
            qDebug() << "\n\n\n   Animation::transform_ModelSet_SKELETONS - OutOfBounds ERROR \n\n\n";
            return;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////////
        ///////////////////////
        touchingJoints.clear();
        ///////////////////////
        ///////////////////////

        /////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////
        int frameNumber = frameNumber_XwrisOffset + motionOffset;
        /////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////


        int currrAdd;
        int childAdd;


        for (int modelID=0; modelID<modelSet.totalModels; modelID++)
        {


                modelSet.models[modelID].skeleton.startingBone = modelSet.models[modelID].skeleton.bones[ 0 ];


                if (modelSet.models[modelID].skeleton.startingBone.childrenVector.size() > 0)
                {

                        for (int j=0; j<modelSet.models[modelID].skeleton.startingBone.childrenVector.size(); j++)
                        {

                                        currrAdd = modelSet.models[modelID].skeleton.startingBone.addressSeListaBones;
                                        childAdd = modelSet.models[modelID].skeleton.startingBone.childrenVector[j];

                                        ///////////////////////////////////////////////////////////
                                        /////////    Wrists/obects - RBM_Joint  ///////////////////
                                        ///////////////////////////////////////////////////////////
                                        calculate_Bones_using_RT( modelID, frameNumber, currrAdd );
                                        ///////////////////////////////////////////////////////////
                                        ///////////////////////////////////////////////////////////

                                        modelSet.models[modelID].skeleton.currentBone = modelSet.models[modelID].skeleton.bones[childAdd];

                                        while (modelSet.models[modelID].skeleton.currentBone.childrenVector.size() > 0)
                                        {

                                                currrAdd = modelSet.models[modelID].skeleton.currentBone.addressSeListaBones;
                                                childAdd = modelSet.models[modelID].skeleton.currentBone.childrenVector[0];

                                                ///////////////////////////////////////////////////////////
                                                calculate_Bones_using_RT( modelID, frameNumber, currrAdd );
                                                ///////////////////////////////////////////////////////////

                                                modelSet.models[modelID].skeleton.currentBone = modelSet.models[modelID].skeleton.bones[childAdd];

                                        }
                                        if (modelSet.models[modelID].skeleton.currentBone.childrenVector.size() == 0 )
                                        {
                                                currrAdd = modelSet.models[modelID].skeleton.currentBone.addressSeListaBones;

                                                ///////////////////////////////////////////////////////////
                                                calculate_Bones_using_RT( modelID, frameNumber, currrAdd );
                                                ///////////////////////////////////////////////////////////
                                        }

                        }

                }
                else
                {
                        currrAdd = modelSet.models[modelID].skeleton.startingBone.addressSeListaBones;

                        calculate_Bones_using_RT( modelID, frameNumber, currrAdd );
                }

        } // for (int modelID=0; modelID<tracker->sequence.posedAnimations[animNumb].modelSet.totalModels; modelID++)

}





void Animation::calculate_Bones_using_RT( int modelID, int frameNumber, int currrAdd ) // WITH Offset ;)
{

                            Eigen::MatrixXd curVect_S( 4,1);
                            Eigen::MatrixXd mat_S(     4,4);
                            Eigen::MatrixXd leafVectt( 4,1);

                            int skkk = modelSet.models[modelID].skeleton.bones[currrAdd].addressSeListaSKINNING;

                            if (skkk>-1)
                            {

                                    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                    skinningBoneStruct *pSkinningBone = &(modelSet.models[modelID].skinningSkeleton.skinningBones[skkk]);
                                    /////////////////////////////////////////////////////////////////////////////////////////////////////


                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                                      leafVectt << modelSet.models[modelID].skeleton.currentBone.length, 0, 0, 1;
                                    if (skkk == 0)    leafVectt <<                                                    0, 0, 0, 1;
                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////

                                    mat_S << movingModels[modelID].motionFrames[ frameNumber ].bones[currrAdd].RT_4x4;

                                    curVect_S << mat_S(0,3), mat_S(1,3), mat_S(2,3), 1;

                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////

                                    pSkinningBone->bone_Start_4x1 =         curVect_S;
                                    pSkinningBone->bone_Leaff_4x1 = mat_S * leafVectt;

                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////

                                    pSkinningBone->bone_Start_PclPointXYZ.x = pSkinningBone->bone_Start_4x1(0);
                                    pSkinningBone->bone_Start_PclPointXYZ.y = pSkinningBone->bone_Start_4x1(1);
                                    pSkinningBone->bone_Start_PclPointXYZ.z = pSkinningBone->bone_Start_4x1(2);
                                    pSkinningBone->bone_Leaff_PclPointXYZ.x = pSkinningBone->bone_Leaff_4x1(0);
                                    pSkinningBone->bone_Leaff_PclPointXYZ.y = pSkinningBone->bone_Leaff_4x1(1);
                                    pSkinningBone->bone_Leaff_PclPointXYZ.z = pSkinningBone->bone_Leaff_4x1(2);

                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    pSkinningBone->bone_Start_isInTouch = false;
                                    pSkinningBone->bone_Leaff_isInTouch = false;
                                    pSkinningBone->addressSeListaTouchingJoints_Start = -666;
                                    pSkinningBone->addressSeListaTouchingJoints_Leaff = -666;

                                    if (modelSet.models[modelID].name.contains("Hand_R")) // red
                                    {
                                        pSkinningBone->bone_Start_RRR = 255;
                                        pSkinningBone->bone_Start_GGG = 0;
                                        pSkinningBone->bone_Start_BBB = 0;
                                    }
                                    else // (modelSet.models[modelID].name.contains("Hand_L")) // blue
                                    {
                                        pSkinningBone->bone_Start_RRR = 0;
                                        pSkinningBone->bone_Start_GGG = 0;
                                        pSkinningBone->bone_Start_BBB = 255;
                                    }
                                    pSkinningBone->bone_Leaff_RRR = pSkinningBone->bone_Start_RRR;
                                    pSkinningBone->bone_Leaff_GGG = pSkinningBone->bone_Start_GGG;
                                    pSkinningBone->bone_Leaff_BBB = pSkinningBone->bone_Start_BBB;
                                    /////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////////////////////////////////////////////////////////////////////////////////////////////////

                            }

}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void Animation::convert_MESH_2_PCL( const int animNumb, const bool colorPerANIM_TRUE__colorPerModel_FALSE )
{

        for (int modelID=0; modelID<modelSet.totalModels; modelID++)
        {

                        //////////////////////////////////////
                        modelSet.models[modelID].PCL->clear();
                        //////////////////////////////////////

                for (int vvv=0; vvv<modelSet.models[modelID].totalVertices; vvv++)
                {

                            pcl::PointXYZRGBNormal  TMP_PointXYZRGBNormal;
                                                    TMP_PointXYZRGBNormal.x = modelSet.models[modelID].mesh.verticesWeighted[vvv](0);
                                                    TMP_PointXYZRGBNormal.y = modelSet.models[modelID].mesh.verticesWeighted[vvv](1);
                                                    TMP_PointXYZRGBNormal.z = modelSet.models[modelID].mesh.verticesWeighted[vvv](2);

                                                    TMP_PointXYZRGBNormal.normal_x = modelSet.models[modelID].mesh.normals_Vertices[vvv](0);
                                                    TMP_PointXYZRGBNormal.normal_y = modelSet.models[modelID].mesh.normals_Vertices[vvv](1);
                                                    TMP_PointXYZRGBNormal.normal_z = modelSet.models[modelID].mesh.normals_Vertices[vvv](2);

                        if (colorPerANIM_TRUE__colorPerModel_FALSE)
                        {
                            if      (animNumb == 0)  {  TMP_PointXYZRGBNormal.r=150; TMP_PointXYZRGBNormal.g=0;   TMP_PointXYZRGBNormal.b=0;    } // red
                            else if (animNumb == 1)  {  TMP_PointXYZRGBNormal.r=0;   TMP_PointXYZRGBNormal.g=0;   TMP_PointXYZRGBNormal.b=150;  } // blue
                        }
                        else
                        {
                            if      (modelID == 0)   {  TMP_PointXYZRGBNormal.r=150; TMP_PointXYZRGBNormal.g=0;   TMP_PointXYZRGBNormal.b=0;    } // red
                            else if (modelID == 1)   {  TMP_PointXYZRGBNormal.r=0;   TMP_PointXYZRGBNormal.g=0;   TMP_PointXYZRGBNormal.b=150;  } // blue
                            else if (modelID == 2)   {  TMP_PointXYZRGBNormal.r=0;   TMP_PointXYZRGBNormal.g=200; TMP_PointXYZRGBNormal.b=200;  } // cyan
                        }

                        /////////////////////////////////////////////////////////////////
                        modelSet.models[modelID].PCL->push_back( TMP_PointXYZRGBNormal );
                        /////////////////////////////////////////////////////////////////
                }

        }

}


