// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>




void Registrator::my_Correspondence_Estimation(              const pcl::PointCloud<TYPE_feature>::Ptr       &source_cloud_FEAT__3D__IN,   // corrEst.setInputSource
                                                             const pcl::PointCloud<TYPE_feature>::Ptr       &target_cloud_FEAT__3D__IN,   // corrEst.setInputTarget
                                                             const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr   &source_cloud_KEYp__3D__IN,
                                                             const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr   &target_cloud_KEYp__3D__IN,
                                                                   QVector<GeometrCORR>                     &corr3D_OBJ_from3D)           // OUT
{

        pcl::CorrespondencesPtr corr_NonFilt_FEAT__3D( new pcl::Correspondences );


        pcl::registration::CorrespondenceEstimation<TYPE_feature,TYPE_feature> corrEst;
        corrEst.setInputSource(            source_cloud_FEAT__3D__IN );
        corrEst.setInputTarget(            target_cloud_FEAT__3D__IN );
        corrEst.determineCorrespondences( *corr_NonFilt_FEAT__3D     );


        corr3D_OBJ_from3D.clear();
        corr3D_OBJ_from3D.resize( corr_NonFilt_FEAT__3D->size() );

        for (int iii=0; iii<      corr_NonFilt_FEAT__3D->size(); iii++)
        {

            TYPE_Point_KEYpt ptSource = source_cloud_KEYp__3D__IN->points[ (*corr_NonFilt_FEAT__3D)[iii].index_query ];
            TYPE_Point_KEYpt ptTarget = target_cloud_KEYp__3D__IN->points[ (*corr_NonFilt_FEAT__3D)[iii].index_match ];

            corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal = ptSource;
            corr3D_OBJ_from3D[iii].source_3D_pt_3f(0)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.x;
            corr3D_OBJ_from3D[iii].source_3D_pt_3f(1)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.y;
            corr3D_OBJ_from3D[iii].source_3D_pt_3f(2)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.z;
            corr3D_OBJ_from3D[iii].source_3D_nrm3f(0)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_x;
            corr3D_OBJ_from3D[iii].source_3D_nrm3f(1)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_y;
            corr3D_OBJ_from3D[iii].source_3D_nrm3f(2)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_z;

            corr3D_OBJ_from3D[iii].target_3D_ptXYZRGBNormal = ptTarget;
            corr3D_OBJ_from3D[iii].target_3D_pt_3f(0)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.x;
            corr3D_OBJ_from3D[iii].target_3D_pt_3f(1)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.y;
            corr3D_OBJ_from3D[iii].target_3D_pt_3f(2)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.z;
            corr3D_OBJ_from3D[iii].target_3D_nrm3f(0)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_x;
            corr3D_OBJ_from3D[iii].target_3D_nrm3f(1)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_y;
            corr3D_OBJ_from3D[iii].target_3D_nrm3f(2)       = corr3D_OBJ_from3D[iii].source_3D_ptXYZRGBNormal.normal_z;

        }

}




void Registrator::my_Correspondence_Filtering_n_Rejection(   const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr     &source_cloud_KEYp__3D__IN,      // corrRej.setInputSource
                                                             const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr     &target_cloud_KEYp__3D__IN,      // corrRej.setInputTarget
                                                             const double                                     &PARAM_CORR_REJ_InlierThreshold, // IN
                                                             const bool                                       &PARAM_CORR_REJ_Kick_1_to_1,     // IN
                                                                   pcl::CorrespondencesPtr                    &corrFUSED_Filt )                // OUT
{

        if (PARAM_CORR_REJ_Kick_1_to_1)
        {
            pcl::registration::CorrespondenceRejectorOneToOne corrRejjj;    // Correspondences with the same match index are removed and
            corrRejjj.setInputCorrespondences(  corrFUSED_Filt        );    // only the one with smallest distance between query and match are kept.
            corrRejjj.getCorrespondences(      *corrFUSED_Filt        );    // That is, considering match->query 1-m correspondences are removed leaving only 1-1 correspondences.
        }

        pcl::registration::CorrespondenceRejectorSampleConsensus<TYPE_Point_KEYpt> corrRej;
        corrRej.setInputSource(               source_cloud_KEYp__3D__IN         );
        corrRej.setInputTarget(               target_cloud_KEYp__3D__IN         );
        corrRej.setInlierThreshold(           PARAM_CORR_REJ_InlierThreshold    );
        corrRej.setMaximumIterations(         PARAM_CORR_REJ_MaximumIterations  );         // Set the maximum number of iterations.
        corrRej.setRefineModel(               true                              );
        corrRej.setInputCorrespondences(      corrFUSED_Filt                    );
        corrRej.getCorrespondences(          *corrFUSED_Filt                    );

}








