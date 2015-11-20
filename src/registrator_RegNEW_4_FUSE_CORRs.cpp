// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
// pcl::Correspondence corr;                          //
//                     corr.weight;                   //
//                     corr.distance;                 //
//                     corr.index_query;    // SOURCE //
//                     corr.index_match;    // TARGET //
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

void Registrator::FUSE_CORRESPONDENCES(       pcl::PointCloud<TYPE_Point_KEYpt>::Ptr    &source_cloud_KEYfused,
                                              pcl::PointCloud<TYPE_Point_KEYpt>::Ptr    &target_cloud_KEYfused,
                                              pcl::CorrespondencesPtr                   &corrFUSED_Filt,
                                              QVector<FUSED_CORRR_label>                &corrFUSED_NonFilt_labels,
                                        const double                                    &IN_PARAM_RUN_CorrWeight_OBJ_Feat2d,
                                        const double                                    &IN_PARAM_RUN_CorrWeight_OBJ_Feat3d,
                                        const double                                    &IN_PARAM_RUN_CorrWeight_SKIN,
                                        const bool                                      &TEST_MODE,
                                              int                                       &KEYfusedID,
                                        const bool                                      &PARAM_RUN__ONLY__TOUCH__TRANSF_IN )
{


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////  CORRESP-FUSION -- OBJECT -- FEAT 2d  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (   (PARAM_RUN__ONLY__TOUCH__TRANSF_IN==false  && TEST_MODE==false && IN_PARAM_RUN_CorrWeight_OBJ_Feat2d>0)                                        ||
               (PARAM_RUN__ONLY__TOUCH__TRANSF_IN==false  && TEST_MODE==true  && IN_PARAM_RUN_CorrWeight_OBJ_Feat2d>0 && PARAM_DBG_Apply_Transf_Obj_Feat2d)    )
        {

                for (int mmm=0; mmm<corr_FEAT__2D.size(); mmm++)
                {

                        TYPE_Point_KEYpt                  source_KEY_toFUSE = corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal;
                        TYPE_Point_KEYpt                  target_KEY_toFUSE = corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal;
                        source_cloud_KEYfused->push_back( source_KEY_toFUSE );
                        target_cloud_KEYfused->push_back( target_KEY_toFUSE );


                        pcl::Correspondence        corr_FEAT__2D__TMP( KEYfusedID, KEYfusedID, IN_PARAM_RUN_CorrWeight_OBJ_Feat2d );
                        corrFUSED_Filt->push_back( corr_FEAT__2D__TMP );


                        FUSED_CORRR_label                   fused_corr_label_TMP;
                                                            fused_corr_label_TMP.label_ID  = 2;
                                                            fused_corr_label_TMP.label_Str = "FEAT2D";
                        corrFUSED_NonFilt_labels.push_back( fused_corr_label_TMP );

                        /////////////
                        KEYfusedID++;
                        /////////////

                }

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////  CORRESP-FUSION -- OBJECT -- FEAT 3d  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (   (PARAM_RUN__ONLY__TOUCH__TRANSF_IN==false  && TEST_MODE==false && IN_PARAM_RUN_CorrWeight_OBJ_Feat3d>0)                                        ||
               (PARAM_RUN__ONLY__TOUCH__TRANSF_IN==false  && TEST_MODE==true  && IN_PARAM_RUN_CorrWeight_OBJ_Feat3d>0 && PARAM_DBG_Apply_Transf_Obj_Feat3d)    )
        {

                for (int mmm=0; mmm<corr_FEAT__3D.size(); mmm++)
                {

                        TYPE_Point_KEYpt                  source_KEY_toFUSE = corr_FEAT__3D[mmm].source_3D_ptXYZRGBNormal;
                        TYPE_Point_KEYpt                  target_KEY_toFUSE = corr_FEAT__3D[mmm].target_3D_ptXYZRGBNormal;
                        source_cloud_KEYfused->push_back( source_KEY_toFUSE );
                        target_cloud_KEYfused->push_back( target_KEY_toFUSE );


                        pcl::Correspondence        corr_FEAT__3D__TMP( KEYfusedID, KEYfusedID, PARAM_RUN_CorrWeight_OBJ_Feat3d );
                        corrFUSED_Filt->push_back( corr_FEAT__3D__TMP );


                        FUSED_CORRR_label                   fused_corr_label_TMP;
                                                            fused_corr_label_TMP.label_ID  = 3;
                                                            fused_corr_label_TMP.label_Str = "FEAT3D";
                        corrFUSED_NonFilt_labels.push_back( fused_corr_label_TMP );

                        /////////////
                        KEYfusedID++;
                        /////////////

                }

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////  CORRESP-FUSION -- HANDS-SKIN  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (   (TEST_MODE==false && IN_PARAM_RUN_CorrWeight_SKIN>0                                 && applyHANDs)  ||
               (TEST_MODE==true  && IN_PARAM_RUN_CorrWeight_SKIN>0 && PARAM_DBG_Apply_Transf_Skin  && applyHANDs)   )
        {

                for (int sss=0; sss<touch->myTouchCorrespSKIN.size(); sss++)
                {

                        TYPE_Point_KEYpt                  source_KEY_toFUSE = touch->myTouchCorrespSKIN[sss].source.vertex_XYZRGBNormal;
                        TYPE_Point_KEYpt                  target_KEY_toFUSE = touch->myTouchCorrespSKIN[sss].target.vertex_XYZRGBNormal;
                        source_cloud_KEYfused->push_back( source_KEY_toFUSE );
                        target_cloud_KEYfused->push_back( target_KEY_toFUSE );


                        pcl::Correspondence        corr_TOUCH_TMP( KEYfusedID, KEYfusedID, PARAM_RUN_CorrWeight_SKIN );
                        corrFUSED_Filt->push_back( corr_TOUCH_TMP );


                        FUSED_CORRR_label                   fused_corr_label_TMP;
                                                            fused_corr_label_TMP.label_ID  = 1;
                                                            fused_corr_label_TMP.label_Str = "TOUCH";
                        corrFUSED_NonFilt_labels.push_back( fused_corr_label_TMP );

                        /////////////
                        KEYfusedID++;
                        /////////////

                }

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


}


