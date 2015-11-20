// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "renderer.h"

Renderer::Renderer()
{
}


Renderer::Renderer( Sequence* sequence )
{
    this->sequence = sequence;
}



void Renderer::renderSkeletons( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int animNumb )
{


                    for (int modelID=0; modelID<sequence->posedAnimations[animNumb].modelSet.totalModels; modelID++)
                    {

                            /////////////////////////////////////////////////////////////////////////////////////////////////////
                            /////////////////////////////////////////////////////////////////////////////////////////////////////
                            if (sequence->posedAnimations[animNumb].modelSet.models[modelID].isRenderable == false)     continue;
                            /////////////////////////////////////////////////////////////////////////////////////////////////////
                            /////////////////////////////////////////////////////////////////////////////////////////////////////


                            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            Model                       *pModel            = &(sequence->posedAnimations[animNumb].modelSet.models[modelID]);
                            SkinningBonesSkeletonStruct *pSkinningSkeleton = &(sequence->posedAnimations[animNumb].modelSet.models[modelID].skinningSkeleton);
                            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                            for (int skkk=0; skkk<pModel->totalSkinningBones; skkk++)
                            {


                                        /////////////////////////////////////////////////////////////////////
                                        /////////////////////////////////////////////////////////////////////
                                        if (pModel->skinningSkeleton.skinningBones.size() == 1)     continue;
                                        if (skkk == 0)                                              continue;
                                        /////////////////////////////////////////////////////////////////////
                                        /////////////////////////////////////////////////////////////////////


                                        //////////////////////////////////////////////////////////////////////////////
                                        skinningBoneStruct *pSkinningBone = &(pSkinningSkeleton->skinningBones[skkk]);
                                        //////////////////////////////////////////////////////////////////////////////


                                        /////////////////////////////////////////////////////////////////////////////
                                        /////////////////////////////////////////////////////////////////////////////
                                        bool                                                 renderEndEff    = false;
                                        if (  (pSkinningBone->name.contains("_2")     ||
                                               pSkinningBone->name.contains("4_1") )  )      renderEndEff    = true;
                                        /////////////////////////////////////////////////////////////////////////////
                                        /////////////////////////////////////////////////////////////////////////////
                                        bool                                                 renderNonEndEff = true;
                                        /////////////////////////////////////////////////////////////////////////////
                                        /////////////////////////////////////////////////////////////////////////////

                                        std::string strrrJoint  = "anim" + QString::number(animNumb).toStdString() + "__JOINT__" + sequence->posedAnimations[animNumb].name.toStdString() + "__" + sequence->posedAnimations[animNumb].modelSet.models[modelID].name.toStdString() + "__" + pSkinningBone->name.toStdString();
                                        std::string strrrEndEff = "anim" + QString::number(animNumb).toStdString() + "__JOINT__" + sequence->posedAnimations[animNumb].name.toStdString() + "__" + sequence->posedAnimations[animNumb].modelSet.models[modelID].name.toStdString() + "__" + pSkinningBone->name.toStdString() + "__EndEffector";
                                        std::string strrrBone   = "anim" + QString::number(animNumb).toStdString() + "__BONE___" + sequence->posedAnimations[animNumb].name.toStdString() + "__" + sequence->posedAnimations[animNumb].modelSet.models[modelID].name.toStdString() + "__" + pSkinningBone->name.toStdString();

                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                        double radius = 1.5;

                                        if (renderNonEndEff)  viewer->addSphere( pSkinningBone->bone_Start_PclPointXYZ, radius,                                 pSkinningBone->bone_Start_RRR,  pSkinningBone->bone_Start_GGG,  pSkinningBone->bone_Start_BBB, strrrJoint, 0 ); /*JOINT*/
                                        if (renderEndEff)     viewer->addSphere( pSkinningBone->bone_Leaff_PclPointXYZ, radius,                                 pSkinningBone->bone_Leaff_RRR,  pSkinningBone->bone_Leaff_GGG,  pSkinningBone->bone_Leaff_BBB, strrrEndEff,0 ); /*END_EFFECTOR*/
                                                              viewer->addLine(   pSkinningBone->bone_Start_PclPointXYZ, pSkinningBone->bone_Leaff_PclPointXYZ,                              0,                              0,                            255, strrrBone,  0 ); /*BONE */
                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                            } // for (int skkk=0; skkk<pModel->totalSkinningBones; skkk++)


                    } // for (int modelID=0; modelID<modelSet.totalModels; modelID++)


}




//////////////////////////////////////////
//////////////////////////////////////////
//////////  convert_MESH_2_PCL  //////////
//////////////////////////////////////////
//////////////////////////////////////////

void Renderer::renderMeshes_2_PCL( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int  animNumb,
                                                                                                 bool renderHands,
                                                                                                 bool renderObject,
                                                                                                 bool renderNormals )
{

        std::cout << "\n\n" << "Renderer::renderMeshes_2_PCL" << "   \t" << "renderHands="<<renderHands
                                                              << "   \t" << "renderObject="<<renderObject
                                                              << "   \t" << "renderNormals="<<renderNormals<< "\n" << std::endl;


        for (int modelID=0; modelID<sequence->posedAnimations[animNumb].modelSet.models.size(); modelID++)
        {

                ////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////
                Model* pModel = &(sequence->posedAnimations[animNumb].modelSet.models[modelID]);
                ////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////


                /////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////////////////////////////////////////
                if (sequence->posedAnimations[animNumb].modelSet.models[modelID].isRenderable == false)     continue;
                /////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////////////////////////////////////////


                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////
                if ( renderObject                  == false &&                                 /////////
                     pModel->name.contains("Hand") == false  )                                 continue;
                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////


                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////
                if ( renderHands                   == false &&                                 /////////
                     pModel->name.contains("Hand") == true   )                                 continue;
                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////


                QString name  = "cloud_meshPCL___anim" + QString::number(animNumb) + "_modelID_" + QString::number(modelID);


                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////

                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>visColor( sequence->posedAnimations[animNumb].modelSet.models[modelID].PCL );

                viewer->addPointCloud<pcl::PointXYZRGBNormal>(sequence->posedAnimations[animNumb].modelSet.models[modelID].PCL,visColor,name.toStdString());

                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,name.toStdString());

                std::cout << "\n\n" << "Renderer::renderMeshes_2_PCL" << "   \t" << "ADDED hand mesh !!!" << std::endl;

                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////

                //////////////////
                if (renderNormals)   viewer->addPointCloudNormals<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>(sequence->posedAnimations[animNumb].modelSet.models[modelID].PCL, sequence->posedAnimations[animNumb].modelSet.models[modelID].PCL, 20, 10.05, QString(name+"_Normals").toStdString());
                //////////////////
        }

}

