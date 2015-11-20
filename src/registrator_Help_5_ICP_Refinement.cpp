// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>




Eigen::Matrix4f Registrator::My_ICP_Refinement___p2plane( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                          pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                          double                                   PARAM_RansacOutlierRejectionThreshold,
                                                          double                                   PARAM_TransformationEpsilon,
                                                          double                                   PARAM_EuclideanFitnessEpsilon,
                                                          int                                      PARAM_MAX_ITER)
{

                                                          Eigen::Matrix4f                          fineTransformation = Eigen::Matrix4f::Identity();
                                                          Eigen::Matrix4f                          guess              = Eigen::Matrix4f::Identity();

        fineTransformation = My_ICP_Refinement___p2plane(                                          source_cloud_UNorg_PN,
                                                                                                   target_cloud_UNorg_PN,
                                                                                                   PARAM_RansacOutlierRejectionThreshold,
                                                                                                   PARAM_TransformationEpsilon,
                                                                                                   PARAM_EuclideanFitnessEpsilon,
                                                                                                   PARAM_MAX_ITER,
                                                                                                   guess );

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Eigen::Matrix4f Registrator::My_ICP_Refinement___p2plane( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                          pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                          double                                   PARAM_RansacOutlierRejectionThreshold,
                                                          double                                   PARAM_TransformationEpsilon,
                                                          double                                   PARAM_EuclideanFitnessEpsilon,
                                                          int                                      PARAM_MAX_ITER,
                                                          Eigen::Matrix4f                          guess)
{

        std::cout << "ICP" << "\n" << std::endl;

        pcl::IterativeClosestPointWithNormals<                           TYPE_Point_PostPr,TYPE_Point_PostPr> icp;
        typedef pcl::registration::TransformationEstimationPointToPlane< TYPE_Point_PostPr,TYPE_Point_PostPr> ICP_TYPE;   // this is the default
        boost::shared_ptr<ICP_TYPE> ICP_TranformationEstimation (        new ICP_TYPE );

        double THRRRR = PARAM_RansacOutlierRejectionThreshold;

            icp.setTransformationEstimation(        ICP_TranformationEstimation );
            icp.setInputSource(                     source_cloud_UNorg_PN );
            icp.setInputTarget(                     target_cloud_UNorg_PN );
            icp.setRANSACOutlierRejectionThreshold( THRRRR ); // inlier, if the distance between the target data index and the transformed source index is smaller than the given inlier distance threshold // default 0.05m
            icp.setMaxCorrespondenceDistance(       THRRRR ); // If the distance is larger than this threshold, the points will be ignored in the alignment process.
            icp.setRANSACIterations(                PARAM_MAX_ITER  );
            icp.setMaximumIterations(               PARAM_MAX_ITER  );
            icp.setTransformationEpsilon(           PARAM_TransformationEpsilon    ); // Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
            icp.setEuclideanFitnessEpsilon(         PARAM_EuclideanFitnessEpsilon  ); // Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. // The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.

            pcl::PointCloud<TYPE_Point_PostPr> outAlignedICP;
            //////////////////////////////////
            icp.align( outAlignedICP, guess );
            //////////////////////////////////


        Eigen::Matrix4f fineTransformation = Eigen::Matrix4f::Identity();


        if   (icp.hasConverged() == false)     ;//std::cout << std::endl << "ICP not coverged!!!" << std::endl;
        else {                                  //std::cout << std::endl << "ICP coverged!!!"     << std::endl;

                //std::cout << std::endl << "ICP - FitnessScore    - " <<   icp.getFitnessScore();
                //std::cout << std::endl << "ICP - getIndices.size - " << (*icp.getIndices()).size() << std::endl << std::endl;

                //////////////////////////////////////////////////
                fineTransformation = icp.getFinalTransformation();
                //////////////////////////////////////////////////
        }

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



Eigen::Matrix4f Registrator::My_ICP_Refinement___p2planeW( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                           pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                           double                                   PARAM_RansacOutlierRejectionThreshold,
                                                           double                                   PARAM_TransformationEpsilon,
                                                           double                                   PARAM_EuclideanFitnessEpsilon,
                                                           int                                      PARAM_MAX_ITER)
{

                                                           Eigen::Matrix4f                          fineTransformation = Eigen::Matrix4f::Identity();
                                                           Eigen::Matrix4f                          guess              = Eigen::Matrix4f::Identity();

        fineTransformation = My_ICP_Refinement___p2planeW(                                          source_cloud_UNorg_PN,
                                                                                                    target_cloud_UNorg_PN,
                                                                                                    PARAM_RansacOutlierRejectionThreshold,
                                                                                                    PARAM_TransformationEpsilon,
                                                                                                    PARAM_EuclideanFitnessEpsilon,
                                                                                                    PARAM_MAX_ITER,
                                                                                                    guess );

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Eigen::Matrix4f Registrator::My_ICP_Refinement___p2planeW( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                           pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                           double                                   PARAM_RansacOutlierRejectionThreshold,
                                                           double                                   PARAM_TransformationEpsilon,
                                                           double                                   PARAM_EuclideanFitnessEpsilon,
                                                           int                                      PARAM_MAX_ITER,
                                                           Eigen::Matrix4f                          guess)
{

        std::cout << "ICP" << "\n" << std::endl;

        pcl::IterativeClosestPointWithNormals<                                   TYPE_Point_PostPr,TYPE_Point_PostPr> icp;
        typedef pcl::registration::TransformationEstimationPointToPlaneWeighted< TYPE_Point_PostPr,TYPE_Point_PostPr> ICP_TYPE;   // this is the default
        boost::shared_ptr<ICP_TYPE> ICP_TranformationEstimation (        new ICP_TYPE );

        double THRRRR = PARAM_RansacOutlierRejectionThreshold;

            icp.setTransformationEstimation(        ICP_TranformationEstimation );
            icp.setInputSource(                     source_cloud_UNorg_PN );
            icp.setInputTarget(                     target_cloud_UNorg_PN );
            icp.setRANSACOutlierRejectionThreshold( THRRRR ); // inlier, if the distance between the target data index and the transformed source index is smaller than the given inlier distance threshold // default 0.05m
            icp.setMaxCorrespondenceDistance(       THRRRR ); // If the distance is larger than this threshold, the points will be ignored in the alignment process.
            icp.setRANSACIterations(                PARAM_MAX_ITER  );
            icp.setMaximumIterations(               PARAM_MAX_ITER  );
            icp.setTransformationEpsilon(           PARAM_TransformationEpsilon    ); // Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
            icp.setEuclideanFitnessEpsilon(         PARAM_EuclideanFitnessEpsilon  ); // Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. // The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.

            pcl::PointCloud<TYPE_Point_PostPr> outAlignedICP;
            //////////////////////////////////
            icp.align( outAlignedICP, guess );
            //////////////////////////////////


        Eigen::Matrix4f fineTransformation = Eigen::Matrix4f::Identity();


        if   (icp.hasConverged() == false)     ;//std::cout << std::endl << "ICP not coverged!!!" << std::endl;
        else {                                  //std::cout << std::endl << "ICP coverged!!!"     << std::endl;

                //std::cout << std::endl << "ICP - FitnessScore    - " <<   icp.getFitnessScore();
                //std::cout << std::endl << "ICP - getIndices.size - " << (*icp.getIndices()).size() << std::endl << std::endl;

                //////////////////////////////////////////////////
                fineTransformation = icp.getFinalTransformation();
                //////////////////////////////////////////////////
        }

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



Eigen::Matrix4f Registrator::My_ICP_Refinement___p2p( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                      pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                      double                                   PARAM_RansacOutlierRejectionThreshold,
                                                      double                                   PARAM_TransformationEpsilon,
                                                      double                                   PARAM_EuclideanFitnessEpsilon,
                                                      int                                      PARAM_MAX_ITER)
{

                                                      Eigen::Matrix4f                          fineTransformation = Eigen::Matrix4f::Identity();
                                                      Eigen::Matrix4f                          guess              = Eigen::Matrix4f::Identity();

        fineTransformation = My_ICP_Refinement___p2p(                                          source_cloud_UNorg_PN,
                                                                                               target_cloud_UNorg_PN,
                                                                                               PARAM_RansacOutlierRejectionThreshold,
                                                                                               PARAM_TransformationEpsilon,
                                                                                               PARAM_EuclideanFitnessEpsilon,
                                                                                               PARAM_MAX_ITER,
                                                                                               guess);

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Eigen::Matrix4f Registrator::My_ICP_Refinement___p2p( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                      pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                      double                                   PARAM_RansacOutlierRejectionThreshold,
                                                      double                                   PARAM_TransformationEpsilon,
                                                      double                                   PARAM_EuclideanFitnessEpsilon,
                                                      int                                      PARAM_MAX_ITER,
                                                      Eigen::Matrix4f                         &guess)
{

        std::cout << "ICP" << "\n" << std::endl;

        pcl::IterativeClosestPointWithNormals<                           TYPE_Point_PostPr,TYPE_Point_PostPr> icp;
        typedef pcl::registration::TransformationEstimationSVD<          TYPE_Point_PostPr,TYPE_Point_PostPr> ICP_TYPE;
        boost::shared_ptr<ICP_TYPE> ICP_TranformationEstimation (        new ICP_TYPE );

        double THRRRR = PARAM_RansacOutlierRejectionThreshold;

            icp.setTransformationEstimation(        ICP_TranformationEstimation );
            icp.setInputSource(                     source_cloud_UNorg_PN );
            icp.setInputTarget(                     target_cloud_UNorg_PN );
            icp.setRANSACOutlierRejectionThreshold( THRRRR ); // inlier, if the distance between the target data index and the transformed source index is smaller than the given inlier distance threshold // default 0.05m
            icp.setMaxCorrespondenceDistance(       THRRRR ); // If the distance is larger than this threshold, the points will be ignored in the alignment process.
            icp.setRANSACIterations(                PARAM_MAX_ITER  );
            icp.setMaximumIterations(               PARAM_MAX_ITER  );
            icp.setTransformationEpsilon(           PARAM_TransformationEpsilon    ); // Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
            icp.setEuclideanFitnessEpsilon(         PARAM_EuclideanFitnessEpsilon  ); // Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. // The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.

            pcl::PointCloud<TYPE_Point_PostPr> outAlignedICP;
            //////////////////////////////////
            icp.align( outAlignedICP, guess );
            //////////////////////////////////


        Eigen::Matrix4f fineTransformation = Eigen::Matrix4f::Identity();


        if   (icp.hasConverged() == false)     ;//std::cout << std::endl << "ICP not coverged!!!" << std::endl;
        else {                                  //std::cout << std::endl << "ICP coverged!!!"     << std::endl;

                //std::cout << std::endl << "ICP - FitnessScore    - " <<   icp.getFitnessScore();
                //std::cout << std::endl << "ICP - getIndices.size - " << (*icp.getIndices()).size() << std::endl << std::endl;

                //////////////////////////////////////////////////
                fineTransformation = icp.getFinalTransformation();
                //////////////////////////////////////////////////
        }

        //////////////////////////
        return fineTransformation;
        //////////////////////////

}



