// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>



Eigen::Vector3f Registrator::myCentroid( const pcl::PointCloud< TYPE_Point_PostPr>::Ptr& cloud )
{
    Eigen::Vector4f                 centroid4f;
    pcl::compute3DCentroid( *cloud, centroid4f );
    Eigen::Vector3f                 centroid3f( centroid4f(0),
                                                centroid4f(1),
                                                centroid4f(2));
    return                          centroid3f;
}
Eigen::Vector3f Registrator::myCentroid( const pcl::PointCloud< TYPE_Point_Sensor>::Ptr& cloud )
{
    Eigen::Vector4f                 centroid4f;
    pcl::compute3DCentroid( *cloud, centroid4f );
    Eigen::Vector3f                 centroid3f( centroid4f(0),
                                                centroid4f(1),
                                                centroid4f(2));
    return                          centroid3f;
}


bool Registrator::reprojectPoint( const TYPE_Point_PostPr &pt, int &u, int &v )
{
    TYPE_Point_Sensor pT;
                      pT.x = pt.x;
                      pT.y = pt.y;
                      pT.z = pt.z;

    return        reprojectPoint(                          pT,      u,      v );
}

bool Registrator::reprojectPoint( const TYPE_Point_Sensor &pt, int &u, int &v )
{
    u = (pt.x * PARAM_CAM_Intrinsics_fx / pt.z) + PARAM_CAM_Intrinsics_px;
    v = (pt.y * PARAM_CAM_Intrinsics_fy / pt.z) + PARAM_CAM_Intrinsics_py;

    return (!pcl_isnan (pt.z) && pt.z > 0 && u >= 0 && u < PARAM_TSDF_Img_Width && v >= 0 && v < PARAM_TSDF_Img_Height);
}



void Registrator::potentialUnitChange_PCL_m2mm( pcl::PointCloud<TYPE_Point_Sensor>::Ptr& cloud_ORGan_P_ )
{

    TYPE_Point_Sensor                        pointMIN, pointMAX;
    pcl::getMinMax3D(       *cloud_ORGan_P_, pointMIN, pointMAX );

    if ( qAbs(pointMIN.z) < PARAM_THRESH_CONV_m2mm ||
         qAbs(pointMAX.z) < PARAM_THRESH_CONV_m2mm  )
    {
        std::cout <<                                                  std::endl;
        std::cout << "potentialUnitChange_PCL_m2mm - CHANGEEE !!!" << std::endl;
        std::cout <<                                                  std::endl;

        for     (int iii=0; iii<cloud_ORGan_P_->height; ++iii)
        {   for (int jjj=0; jjj<cloud_ORGan_P_->width;  ++jjj)
            {
                if (  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).x ) == false  &&
                      pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).y ) == false  &&
                      pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).z ) == false   )
                {
                    /////////////////////////////////////
                    (*cloud_ORGan_P_)(jjj,iii).x *= 1000;
                    (*cloud_ORGan_P_)(jjj,iii).y *= 1000;
                    (*cloud_ORGan_P_)(jjj,iii).z *= 1000;
                    /////////////////////////////////////
                }
            }
        }
    }

}



void Registrator::restore_NANs___dueToPCLbugInBILATERAL( pcl::PointCloud<TYPE_Point_PostPr>::Ptr& cloud_ORGan_P_, bool shouldPrint_countYesN_NANs )
{
    if (shouldPrint_countYesN_NANs)     std::cout << "restore_NANs___dueToPCLbugInBILATERAL" << "\t\t" << "cloud_ORGan_P_->height = " << cloud_ORGan_P_->height << std::endl;
    if (shouldPrint_countYesN_NANs)     std::cout << "restore_NANs___dueToPCLbugInBILATERAL" << "\t\t" << "cloud_ORGan_P_->width  = " << cloud_ORGan_P_->width  << std::endl;

    double minZ = +std::numeric_limits<float>::infinity();
    double maxZ = -std::numeric_limits<float>::infinity();

    ////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat testNANsIMG = cv::Mat(cloud_ORGan_P_->height,cloud_ORGan_P_->width,CV_8UC1,255);
    ////////////////////////////////////////////////////////////////////////////////////////
    int countNANs_YES = 0;
    int countNANs_NOO = 0;
    //////////////////////
    for     (int iii=0; iii<cloud_ORGan_P_->height; ++iii)
    {   for (int jjj=0; jjj<cloud_ORGan_P_->width;  ++jjj)
        {
            ///////////////////////////////////////////////////
            if (  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).x )  ||
                  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).y )  ||
                  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).z )   )
            ///////////////////////////////////////////////////
            {
                (*cloud_ORGan_P_)(jjj,iii).x = std::numeric_limits<float>::quiet_NaN();
                (*cloud_ORGan_P_)(jjj,iii).y = std::numeric_limits<float>::quiet_NaN();
                (*cloud_ORGan_P_)(jjj,iii).z = std::numeric_limits<float>::quiet_NaN();

                countNANs_YES++;
                testNANsIMG.at<uchar>(iii,jjj) = 0;
            }
            else
            {
                countNANs_NOO++;
            }

            if (minZ > (*cloud_ORGan_P_)(jjj,iii).z)
                minZ = (*cloud_ORGan_P_)(jjj,iii).z;

            if (maxZ < (*cloud_ORGan_P_)(jjj,iii).z)
                maxZ = (*cloud_ORGan_P_)(jjj,iii).z;
        }
    }
    if (shouldPrint_countYesN_NANs)     std::cout << "testNANs - yes NANs" << "\t\t" << countNANs_YES << "\t\t" << "minZ="<<minZ << "\t\t" << "maxZ="<<maxZ << std::endl;
    if (shouldPrint_countYesN_NANs)     std::cout << "testNANs - noo NANs" << "\t\t" << countNANs_NOO << "\t\t" << "minZ="<<minZ << "\t\t" << "maxZ="<<maxZ << std::endl << std::endl;
}


void Registrator::myLoadCameraParams( boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer )
{

        CameraSet* pCameraSet = &(sequence->cameraSet);

        ////////////////////////////////////////
        ////  Coordinate Frame Conventions  ////
        ////////////////////////////////////////
        // PCL    - http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions
        // OpenGL - http://www.falloutsoftware.com//tutorials/gl/view.gif

        if (syntheticORrealistic == "synthetic")
        {

                if (PARAM_DBG_cameraFlipX)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,0) *= -1; // Useless
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,0) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,0) *= -1;
                }

                if (PARAM_DBG_cameraFlipY)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,1) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,1) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,1) *= -1;
                }

                if (PARAM_DBG_cameraFlipZ)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,2) *= -1; // Only for Video/Screenshot -> mirrors viewer to resemble original image
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,2) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,2) *= -1;
                }

        }
        else
        {

                if (PARAM_DBG_cameraFlipX)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,0) *= -1; // Useless
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,0) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,0) *= -1;
                }

                if (PARAM_DBG_cameraFlipY)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,1) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,1) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,1) *= -1;
                }

                if (PARAM_DBG_cameraFlipZ)
                {
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(0,2) *= -1; // Only for Video/Screenshot -> mirrors viewer to resemble original image
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(1,2) *= -1;
                    pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4(2,2) *= -1;
                }

        }

        viewer->setCameraParameters(   (pCameraSet->cameras[ pCameraSet->currentCameraID ].KKK   ).cast<float>(),
                                       (pCameraSet->cameras[ pCameraSet->currentCameraID ].RT_4x4).cast<float>()   );

        viewer->setSize( PARAM_Viewer_PNG_Size_WWW*1,
                         PARAM_Viewer_PNG_Size_HHH );

}







void Registrator::texture3Dpoints_TRANSFORM_Source( Eigen::Matrix4f transformationMatrix4f )
{

        Eigen::Affine3f transf_(transformationMatrix4f); // O.K. // transf_.matrix() == transf

        for (int mmm=0; mmm<corr_FEAT__2D.size(); mmm++)
        {
            corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal = pcl::transformPoint( corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal, transf_ );
            corr_FEAT__2D[mmm].source_3D_pt_3f <<                              corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal.x,
                                                                               corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal.y,
                                                                               corr_FEAT__2D[mmm].source_3D_ptXYZRGBNormal.z;
        }

}



void Registrator::texture3Dpoints_TRANSFORM_Target( Eigen::Matrix4f transformationMatrix4f )
{

        Eigen::Affine3f transf_(transformationMatrix4f);

        for (int mmm=0; mmm<corr_FEAT__2D.size(); mmm++)
        {
            corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal = pcl::transformPoint( corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal, transf_ );
            corr_FEAT__2D[mmm].target_3D_pt_3f <<                              corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal.x,
                                                                               corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal.y,
                                                                               corr_FEAT__2D[mmm].target_3D_ptXYZRGBNormal.z;
        }

}



cv::Mat Registrator::getMASK_denoised_fromPCL___handsYES( const pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_ )
{

        ////////////////////////////////////////////////////////////////////////////////////////
        cv::Mat mask = cv::Mat::zeros( cloud_ORGan_P_->height, cloud_ORGan_P_->width, CV_8UC1 );
        ////////////////////////////////////////////////////////////////////////////////////////
        for (int iii=0; iii<cloud_ORGan_P_->points.size(); iii++)
        {

                if ( pcl_isnan(cloud_ORGan_P_->points[iii].x) ||
                     pcl_isnan(cloud_ORGan_P_->points[iii].y) ||
                     pcl_isnan(cloud_ORGan_P_->points[iii].z)  )    continue;

                Eigen::Vector3d vertex3d( cloud_ORGan_P_->points[iii].x,
                                          cloud_ORGan_P_->points[iii].y,
                                          cloud_ORGan_P_->points[iii].z);

                Eigen::Vector2d proj2d = sequence->cameraSet.project__3D_Vertex__to__2D_Point( vertex3d, 0 );

                cv::Point2d pt2d( proj2d(0),    // 0 - jx
                                  proj2d(1));   // 1 - iy

                mask.at<uchar>( pt2d ) = 255;

        }
                                                                            //  PARAM_MORPH_ErodeSize
        cv::erode(   mask, mask, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(3,3) ) ); // thinner ;) // kick noise
        cv::dilate(  mask, mask, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(3,3) ) ); // fatter  ;) // kick noise

        cv::dilate(  mask, mask, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(3,3) ) ); // fatter  ;) // fill holes
        cv::erode(   mask, mask, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(3,3) ) ); // thinner ;) // fill holes
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        cv::erode(   mask, mask, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(3,3) ) ); // thinner ;) // no boundary

        ////////////
        return mask;
        ////////////

}


cv::Mat Registrator::getMASK_denoised_fromPCL___handsNOO( const cv::Mat &depthMap )
{

        ////////////////////////////////////
        cv::Mat mask = depthMap.clone() > 0;
        ////////////////////////////////////

        /////////////
        return  mask;
        /////////////

}





void Registrator::corresp_2_pt_source( const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr &cloud_PN, const pcl::CorrespondencesPtr &corrSet, const int index, Eigen::Vector3f &ptVec3f )
{

        TYPE_Point_KEYpt ptOUT;

                   ptOUT = cloud_PN->points[ (*corrSet)[index].index_query ];
        ptVec3f << ptOUT.x,
                   ptOUT.y,
                   ptOUT.z;

}

void Registrator::corresp_2_pt_target( const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr &cloud_PN, const pcl::CorrespondencesPtr &corrSet, const int index, Eigen::Vector3f &ptVec3f )
{

        TYPE_Point_KEYpt ptOUT;

                   ptOUT = cloud_PN->points[ (*corrSet)[index].index_match ];
        ptVec3f << ptOUT.x,
                   ptOUT.y,
                   ptOUT.z;

}


////////////////////////////////
// source - index_query // OK //
// target - index_match // OK //
////////////////////////////////


void Registrator::readIMGs_getFEAT2D(             const int                                       &sourceID_fileID,
                                                  const int                                       &targetID_fileID,
                                                  const QString                                   &INPUT_PATH,
                                                  const QString                                   &OUTPUT_PATH,
                                                  const bool                                      &FEAT_ENABLED_2d,
                                                  const int                                       &ifSTART_fileID,
                                                  const QString                                   &caller_local_OR_global,
                                                        QVector<TextureCORR>                      &myTextureCORR,
                                                  const bool                                      &debugModeON_fromGlobal,
                                                  const bool                                      &debug_1PAIR_fromGlobal,
                                                  const bool                                      &PRINT_Paths )
{

        pcl::PointCloud<TYPE_Point_Sensor  >::Ptr source_cloud_ORGan_P_(       new pcl::PointCloud<TYPE_Point_Sensor>   );
        pcl::PointCloud<TYPE_Point_Sensor  >::Ptr target_cloud_ORGan_P_(       new pcl::PointCloud<TYPE_Point_Sensor>   );

        readIMGs_getPCLs_getFEAT2D( sourceID_fileID,
                                    targetID_fileID,
                                    INPUT_PATH,
                                    OUTPUT_PATH,
                                    FEAT_ENABLED_2d,
                                    ifSTART_fileID,
                                    caller_local_OR_global,
                                    source_cloud_ORGan_P_,
                                    target_cloud_ORGan_P_,
                                    myTextureCORR,
                                    debugModeON_fromGlobal,
                                    debug_1PAIR_fromGlobal,
                                    PRINT_Paths );

}



void Registrator::readIMGs_getPCLs_asRawAsPossible_kickedOutHandsWhenApplicable( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &CURR_cloud_ORGan_Pn,
                                                                                 QString                                  INPUT_PATH         ,
                                                                                 int                                      fileID             )
{

        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        pcl::PointCloud<TYPE_Point_Sensor>::Ptr     CURR_cloud_ORGan_p_(new pcl::PointCloud<TYPE_Point_Sensor>);
        cv::Mat                                                            imgRGB,   imgDDD;
        readIMG_getPCL(       INPUT_PATH,  fileID,  CURR_cloud_ORGan_p_,   imgRGB,   imgDDD,   false   );
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        my_Normal_Estimator(  CURR_cloud_ORGan_p_,  CURR_cloud_ORGan_Pn, syntheticORrealistic, false   );
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        kickOut_pointsWith_CoordNAN_NormalNAN(                                      CURR_cloud_ORGan_Pn       );
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////

}



void Registrator::readIMGs_getPCLs_getFEAT2D(     const int                                       &sourceID_fileID,
                                                  const int                                       &targetID_fileID,
                                                  const QString                                   &INPUT_PATH,
                                                  const QString                                   &OUTPUT_PATH,
                                                  const bool                                      &FEAT_ENABLED_2d,
                                                  const int                                       &ifSTART_fileID,
                                                  const QString                                   &caller_local_OR_global,
                                                        pcl::PointCloud<TYPE_Point_Sensor  >::Ptr &source_cloud_ORGan_P_,
                                                        pcl::PointCloud<TYPE_Point_Sensor  >::Ptr &target_cloud_ORGan_P_,
                                                        QVector<TextureCORR>                      &myTextureCORR,
                                                  const bool                                      &debugModeON_fromGlobal,
                                                  const bool                                      &debug_1PAIR_fromGlobal,
                                                  const bool                                      &PRINT_Paths )
{


            cv::Mat                                                             source_imgRGB, source_imgDDD_yml;
            cv::Mat                                                             target_imgRGB, target_imgDDD_yml;
            readIMG_getPCL( INPUT_PATH, targetID_fileID, target_cloud_ORGan_P_, target_imgRGB, target_imgDDD_yml, PRINT_Paths );
            readIMG_getPCL( INPUT_PATH, sourceID_fileID, source_cloud_ORGan_P_, source_imgRGB, source_imgDDD_yml, PRINT_Paths );


            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            if ( FEAT_ENABLED_2d )
            {

                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    cv::Mat mask_source;
                    cv::Mat mask_target;

                    if (hasHANDs   &&   syntheticORrealistic.contains("realistic")) // synthetic has already hands kicked out !!!
                    {
                            mask_target = getMASK_denoised_fromPCL___handsYES( target_cloud_ORGan_P_ );
                            mask_source = getMASK_denoised_fromPCL___handsYES( source_cloud_ORGan_P_ );
                    }
                    else // also for --> // if (syntheticORrealistic == "synthetic")
                    {
                            mask_target = getMASK_denoised_fromPCL___handsNOO( target_imgDDD_yml );
                            mask_source = getMASK_denoised_fromPCL___handsNOO( source_imgDDD_yml );
                    }

                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    cv::Mat target_IMG_;
                    cv::Mat source_IMG_;

                    if (PARAM_DBG_OBJ_FEAT_2D_FLIP)
                    {
                            cv::transpose( target_imgRGB, target_IMG_ );   cv::cvtColor( target_IMG_, target_IMG_, cv::COLOR_BGR2GRAY );   cv::transpose( mask_target, mask_target );   cv::transpose( target_imgDDD_yml, target_imgDDD_yml );
                            cv::transpose( source_imgRGB, source_IMG_ );   cv::cvtColor( source_IMG_, source_IMG_, cv::COLOR_BGR2GRAY );   cv::transpose( mask_source, mask_source );   cv::transpose( source_imgDDD_yml, source_imgDDD_yml );
                    }
                    else
                    {
                            target_IMG_ = target_imgRGB.clone();
                            source_IMG_ = source_imgRGB.clone();
                    }
                    /////////////////////////////////////////////////////////////
                    /////////////////////////////////////////////////////////////
                    std::vector<cv::KeyPoint>   source_keypoints;
                    std::vector<cv::KeyPoint>   target_keypoints;
                    std::vector<cv::DMatch>     matches;


                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // http://stackoverflow.com/questions/9539473/opencv-orb-not-finding-matches-once-rotation-scale-invariances-are-introduced
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    rmatcher.setRatio(                 PARAM_RMATCHER_Ratio                 );
                    rmatcher.setMinDistanceToEpipolar( PARAM_RMATCHER_MinDistanceToEpipolar );
                    rmatcher.setConfidenceLevel(       PARAM_RMATCHER_ConfidenceLevel       );

                    rmatcher.match( source_IMG_,                            // IN
                                    target_IMG_,                            // IN
                                    mask_source,                            // IN
                                    mask_target,                            // IN
                                    matches,                                // OUT
                                    source_keypoints,                       // OUT
                                    target_keypoints,                       // OUT
                                    caller_local_OR_global,                 // IN
                                    PARAM_CORR_FEAT2D_PerformRansac2D);     // IN


                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    cv::Mat                             source_IMG_SIFT;
                    cv::Mat                             target_IMG_SIFT;
                    cv::Mat                                                                                                                 match_IMG;
                    if (PARAM_DBG_OBJ_FEAT_2D_SHOW || debugModeON_fromGlobal )  {  cv::drawMatches( source_IMG_, source_keypoints,          /////////
                                                                                                    target_IMG_, target_keypoints, matches, match_IMG, cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                        cv::drawKeypoints( source_IMG_, source_keypoints, source_IMG_SIFT, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS ); // DRAW_RICH_KEYPOINTS // NOT_DRAW_SINGLE_POINTS // DRAW_OVER_OUTIMG // http://docs.opencv.org/modules/features2d/doc/drawing_function_of_keypoints_and_matches.html
                        cv::drawKeypoints( target_IMG_, target_keypoints, target_IMG_SIFT, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
                    }
                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                    std::string openCV_windowMatches_NAME_big   = QString(QString("matches_feat2d")).toStdString();
                    std::string openCV_windowMatches_NAME_small = QString(QString("matches_feat2d")+" \t"+QString::number(sourceID_fileID)
                                                                                                   +" \t"+QString::number(targetID_fileID)
                                                                                                   +" \t"+QString::number(matches.size())).toStdString();

                    if (PARAM_DBG_OBJ_FEAT_2D_SHOW || debugModeON_fromGlobal)
                    {
                        if (source_IMG_SIFT.cols > PARAM_TSDF_Img_Width  ||
                            source_IMG_SIFT.rows > PARAM_TSDF_Img_Height)
                        {
                            double scaleDownFactor = 2.08;

                            cv::resize( source_IMG_SIFT, source_IMG_SIFT, cv::Size( source_IMG_SIFT.cols/scaleDownFactor, source_IMG_SIFT.rows/scaleDownFactor ) );
                            cv::resize( target_IMG_SIFT, target_IMG_SIFT, cv::Size( target_IMG_SIFT.cols/scaleDownFactor, target_IMG_SIFT.rows/scaleDownFactor ) );
                            cv::resize(  match_IMG,       match_IMG,      cv::Size(  match_IMG.     cols/scaleDownFactor,  match_IMG.     rows/scaleDownFactor ) );
                        }

                        cv::imshow(  openCV_windowMatches_NAME_big,    match_IMG      );
                        cv::imshow( "source_IMG_SIFT",                source_IMG_SIFT );
                        cv::imshow( "target_IMG_SIFT",                target_IMG_SIFT );
                        std::cout << openCV_windowMatches_NAME_small  <<  std::endl;
                    }
                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                    cv::Mat source_IMG_SIFT_ = source_IMG_.clone();
                    cv::Mat target_IMG_SIFT_ = target_IMG_.clone();


                    //////////////////////
                    //////////////////////
                    myTextureCORR.clear();
                    //////////////////////
                    //////////////////////


                    for (int mmm=0; mmm<matches.size(); mmm++)
                    {

                            cv::Point2d  source_PT = source_keypoints[ matches[mmm].queryIdx ].pt;
                            cv::Point2d  target_PT = target_keypoints[ matches[mmm].trainIdx ].pt;

                            cv::circle ( source_IMG_SIFT_, source_PT, 2, cv::Scalar(0,255,0) );
                            cv::circle ( target_IMG_SIFT_, target_PT, 2, cv::Scalar(0,255,0) );


                            ///////////////////////////////////
                            TextureCORR           textCORR_TMP;
                            ///////////////////////////////////


                            int source_iii = round( source_PT.y );
                            int source_jjj = round( source_PT.x );
                            int target_iii = round( target_PT.y );
                            int target_jjj = round( target_PT.x );


                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            ///////////////////////////////////////////////////////////////// SOS -> ( COLUMN    , ROW        )////////////////////////////
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            textCORR_TMP.source_3D_pt_3f(0)                = 0;
                            textCORR_TMP.source_3D_pt_3f(1)                = 0;
                            textCORR_TMP.source_3D_pt_3f(2)                = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.source_3D_nrm3f(0)                = 0;
                            textCORR_TMP.source_3D_nrm3f(1)                = 0;
                            textCORR_TMP.source_3D_nrm3f(2)                = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.source_3D_ptXYZRGBNormal.x        = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.y        = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.z        = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.source_3D_ptXYZRGBNormal.normal_x = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.normal_y = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.normal_z = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.source_3D_ptXYZRGBNormal.r        = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.g        = 0;
                            textCORR_TMP.source_3D_ptXYZRGBNormal.b        = 0;
                            ////////////////////////////////////////////////////////////
                            textCORR_TMP.source_iii_atORGpcl               = source_iii;
                            textCORR_TMP.source_jjj_atORGpcl               = source_jjj;
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            //////////////////////////////////////////////////////////////////////// SOS -> ( COLUMN    , ROW        )/////////////////////
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            textCORR_TMP.target_3D_pt_3f(0)                = 0;
                            textCORR_TMP.target_3D_pt_3f(1)                = 0;
                            textCORR_TMP.target_3D_pt_3f(2)                = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.target_3D_nrm3f(0)                = 0;
                            textCORR_TMP.target_3D_nrm3f(1)                = 0;
                            textCORR_TMP.target_3D_nrm3f(2)                = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.target_3D_ptXYZRGBNormal.x        = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.y        = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.z        = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.target_3D_ptXYZRGBNormal.normal_x = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.normal_y = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.normal_z = 0;
                            ///////////////////////////////////////////////////
                            textCORR_TMP.target_3D_ptXYZRGBNormal.r        = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.g        = 0;
                            textCORR_TMP.target_3D_ptXYZRGBNormal.b        = 0;
                            ////////////////////////////////////////////////////////////
                            textCORR_TMP.target_iii_atORGpcl               = target_iii;
                            textCORR_TMP.target_jjj_atORGpcl               = target_jjj;
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                            /////////////////////////////////////
                            myTextureCORR.append( textCORR_TMP );
                            /////////////////////////////////////

                    }
                    /////////////////////////////////////////////////////////////
                    /////////////////////////////////////////////////////////////
                    //std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@"       << std::endl;
                    //std::cout << "@@@@@ " << myTextureCORR.size() << std::endl;
                    //std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@"       << std::endl;
                    /////////////////////////////////////////////////////////////
                    /////////////////////////////////////////////////////////////
                    if (PARAM_DBG_OBJ_FEAT_2D_SHOW || debugModeON_fromGlobal)
                    {
                        if (debugModeON_fromGlobal && debug_1PAIR_fromGlobal==false)    cv::waitKey(1);
                        else                                                            cv::waitKey();
                    }

            }

}


void Registrator::refineTextureCORRs_withPostProccessedPCL( const pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_POSTT_PN,
                                                            const pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_POSTT_PN,
                                                                  QVector<TextureCORR>                    &myTextureCORR,
                                                            const bool                                    &TEST_MODE,
                                                            const QString                                 &p2p_p2plane )
{

        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        bool                                    is_p2plane = false;
        if (p2p_p2plane.contains("p2plane"))    is_p2plane = true;
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////


        for (int mmm=myTextureCORR.size()-1; mmm>=0; mmm--)
        {

                ////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////
                int source_iii = myTextureCORR[mmm].source_iii_atORGpcl;
                int source_jjj = myTextureCORR[mmm].source_jjj_atORGpcl;
                ////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////
                int target_iii = myTextureCORR[mmm].target_iii_atORGpcl;
                int target_jjj = myTextureCORR[mmm].target_jjj_atORGpcl;
                ////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////


                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////// SOS -> ( COLUMN    , ROW        )/////////////////////////////      NEIGHBORHOUD AVERAGING ???
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].source_3D_pt_3f(0)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).x;
                myTextureCORR[mmm].source_3D_pt_3f(1)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).y;
                myTextureCORR[mmm].source_3D_pt_3f(2)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).z;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].source_3D_nrm3f(0)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).normal_x;
                myTextureCORR[mmm].source_3D_nrm3f(1)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).normal_y;
                myTextureCORR[mmm].source_3D_nrm3f(2)                = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).normal_z;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.x        = myTextureCORR[mmm].source_3D_pt_3f(0);
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.y        = myTextureCORR[mmm].source_3D_pt_3f(1);
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.z        = myTextureCORR[mmm].source_3D_pt_3f(2);
                /////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_x = myTextureCORR[mmm].source_3D_nrm3f(0);
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_y = myTextureCORR[mmm].source_3D_nrm3f(1);
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_z = myTextureCORR[mmm].source_3D_nrm3f(2);
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.r        = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).r;
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.g        = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).g;
                myTextureCORR[mmm].source_3D_ptXYZRGBNormal.b        = (*source_cloud_POSTT_PN).at( source_jjj, source_iii ).b;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////// SOS -> ( COLUMN    , ROW        )/////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].target_3D_pt_3f(0)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).x;
                myTextureCORR[mmm].target_3D_pt_3f(1)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).y;
                myTextureCORR[mmm].target_3D_pt_3f(2)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).z;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].target_3D_nrm3f(0)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).normal_x;
                myTextureCORR[mmm].target_3D_nrm3f(1)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).normal_y;
                myTextureCORR[mmm].target_3D_nrm3f(2)                = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).normal_z;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.x        = myTextureCORR[mmm].target_3D_pt_3f(0);
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.y        = myTextureCORR[mmm].target_3D_pt_3f(1);
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.z        = myTextureCORR[mmm].target_3D_pt_3f(2);
                /////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_x = myTextureCORR[mmm].target_3D_nrm3f(0);
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_y = myTextureCORR[mmm].target_3D_nrm3f(1);
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_z = myTextureCORR[mmm].target_3D_nrm3f(2);
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.r        = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).r;
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.g        = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).g;
                myTextureCORR[mmm].target_3D_ptXYZRGBNormal.b        = (*target_cloud_POSTT_PN).at( target_jjj, target_iii ).b;
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                if ( (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.x        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.y        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.z        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_x ) && is_p2plane) ||
                     (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_y ) && is_p2plane) ||
                     (pcl_isnan( myTextureCORR[mmm].source_3D_ptXYZRGBNormal.normal_z ) && is_p2plane) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.x        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.y        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.z        )              ) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_x ) && is_p2plane) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_y ) && is_p2plane) ||
                     (pcl_isnan( myTextureCORR[mmm].target_3D_ptXYZRGBNormal.normal_z ) && is_p2plane)  )
                {
                                myTextureCORR.erase(
                                myTextureCORR.begin() + mmm  );

                                if (TEST_MODE)
                                {
                                    std::cout << "refineTextureCORRs_withPostProccessedPCL - deleting " << mmm << std::endl;
                                }
                }

        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}




void Registrator::readIMG_getPCL( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_,                                   const bool PRINT_Paths )
{

                  cv::Mat                                                                                                                        imgRGB,          imgDDD;
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  readIMG_getPCL(                INPUT_PATH,            fileID,                                          CURR_cloud_P_,          imgRGB,          imgDDD,            PRINT_Paths );
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}



void Registrator::readIMG_getPCL( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_, cv::Mat &imgRGB, cv::Mat &imgDDD, const bool PRINT_Paths )
{

                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  readIMG       (                INPUT_PATH,            fileID,                                                                  imgRGB,          imgDDD,            PRINT_Paths );
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        if (syntheticORrealistic.contains("realistic"))
        {
                  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  BackProject_Depth_2_PCL( imgRGB, imgDDD, *CURR_cloud_P_, PARAM_MORPH_ErodeSize, !applyHANDs, PARAM_applySKIN_2_PCL_backProjection );
                  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
        else
        {
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  readPCL       (                INPUT_PATH,            fileID,                                          CURR_cloud_P_,                                              PRINT_Paths );
                  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

}



void Registrator::readPCL       ( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_,                                   const bool PRINT_Paths )
{

    std::string  fileName =  QString(   INPUT_PATH + "pcl/" + QString::number( fileID ).rightJustified(fileNamePADDING,'0',false) + ".pcd"   ).toStdString();

    if (PRINT_Paths)    std::cout << fileName << std::endl;

    pcl::io::loadPCDFile<TYPE_Point_Sensor>( fileName, *CURR_cloud_P_ );

}



void Registrator::readIMG       ( const QString &INPUT_PATH, const int &fileID,                                                         cv::Mat &imgRGB, cv::Mat &imgDDD, const bool PRINT_Paths )
{

    std::string PATH_IMG_rgb         = QString(   INPUT_PATH + "rgb/"       + QString::number( fileID ).rightJustified(fileNamePADDING,'0',false) + ".png"   ).toStdString();
    std::string PATH_IMG_ddd_raw_yml = QString(   INPUT_PATH + "depth/"     + QString::number( fileID ).rightJustified(fileNamePADDING,'0',false) + ".yml"   ).toStdString();
    std::string PATH_IMG_ddd_raw_png = QString(   INPUT_PATH + "depth_png/" + QString::number( fileID ).rightJustified(fileNamePADDING,'0',false) + ".png"   ).toStdString();


    if (PRINT_Paths)    std::cout <<PATH_IMG_rgb << std::endl;


    imgRGB = cv::imread(   PATH_IMG_rgb );
  //imgDDD = load_YML_img( PATH_IMG_ddd_raw_yml );
    imgDDD = cv::imread(   PATH_IMG_ddd_raw_png, CV_LOAD_IMAGE_ANYDEPTH );   imgDDD.convertTo( imgDDD, CV_32FC1 );
    imgDDD.convertTo( imgDDD, CV_32FC1 );

}





Eigen::Matrix4f Registrator::myTransformationEstimationPointToPointWeighted( const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr &source_cloud_KEY,
                                                                             const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr &target_cloud_KEY,
                                                                             const pcl::CorrespondencesPtr                 &corr_Filt )
{

                                                //////////////////////////////////
        pcl::TransformationFromCorrespondences  transformation_from_correspondeces;
                                                transformation_from_correspondeces.reset();
                                                //////////////////////////////////

        for (int mmm=0; mmm<corr_Filt->size(); mmm++)
        {

                TYPE_Point_KEYpt ptSource = source_cloud_KEY->points[ (*corr_Filt)[mmm].index_query ];
                TYPE_Point_KEYpt ptTarget = target_cloud_KEY->points[ (*corr_Filt)[mmm].index_match ];
                float            weighttt =                           (*corr_Filt)[mmm].weight;

                Eigen::Vector3f sourceVec3f( ptSource.x, ptSource.y, ptSource.z );
                Eigen::Vector3f targetVec3f( ptTarget.x, ptTarget.y, ptTarget.z );

                                                //////////////////////////////////
                                                transformation_from_correspondeces.add( sourceVec3f, targetVec3f, weighttt );
                                                //////////////////////////////////

        }

        Eigen::Matrix4f coarseTransformation = transformation_from_correspondeces.getTransformation().matrix();

        ////////////////////////////
        return coarseTransformation;
        ////////////////////////////
}














void Registrator::getRelative__diff_Angle__diff_Trans(                         const Eigen::Matrix4f &Pose_source,
                                                                               const Eigen::Matrix4f &Pose_target,
                                                                                     double          &diff_Angle,
                                                                                     double          &diff_Trans)
    {

                                                      Eigen::Matrix4f                             currPoseREL;

                  getRelative__diff_Angle__diff_Trans__relativeTransf(                            Pose_source,
                                                                                                  Pose_target,
                                                                                                  diff_Angle,
                                                                                                  diff_Trans,
                                                                                                  currPoseREL);
}


void Registrator::getRelative__diff_Angle__diff_Trans__relativeTransf(         const Eigen::Matrix4f &Pose_source,
                                                                               const Eigen::Matrix4f &Pose_target,
                                                                                     double          &diff_Angle,
                                                                                     double          &diff_Trans,
                                                                                     Eigen::Matrix4f &currPoseREL)
{

        currPoseREL = getRelativeTransformation( Pose_source, Pose_target );

        getRelative__diff_Angle__diff_Trans__FROM__relativeTransf( currPoseREL,
                                                                   diff_Angle,
                                                                   diff_Trans);

}


void Registrator::getRelative__diff_Angle__diff_Trans__FROM__relativeTransf(    const Eigen::Matrix4f &currPoseREL,
                                                                                      double          &diff_Angle,
                                                                                      double          &diff_Trans)
{

        Eigen::Matrix3f poseChange_Rottt = currPoseREL.block<3,3>(0,0);
        Eigen::Vector3f poseChange_Trans = currPoseREL.block<3,1>(0,3);

        diff_Angle = Eigen::AngleAxisf( poseChange_Rottt ).angle() * 180 / M_PI;            // rad -> degrees
        diff_Trans =                    poseChange_Trans  .norm();

}



Eigen::Matrix4f Registrator::getRelativeTransformation( const Eigen::Matrix4f& pose_iii, const Eigen::Matrix4f& pose_iii_m1 )
{

        Eigen::Matrix4f pose_NEW = pose_iii * pose_iii_m1.inverse(); // OK   // THIS ALONE GIVES ~finalUpdateMat of the PAIRWISE ALIGNMENT - line ~1020 in LocAlign.cpp
        return          pose_NEW;

}














cv::Mat Registrator::load_YML_img( std::string PATH )
{
    cv::Mat img;
    cv::FileStorage fs( PATH, cv::FileStorage::READ );
    cv::FileNode fn = fs["depth"];
    if (fn.empty())
    {
        std::cout << "\n\n\n" << "EMPTY YML !!!!!!!!! - " << PATH << "\n\n" << std::endl;
        return img;
    }
    cv::FileNodeIterator current = fn.begin(), it_end = fn.end(); // go through the node
    for ( ; current != it_end; current++)
    {
        cv::FileNode item = *current;
        item >> img;
    }
    return img;
}












void Registrator::transf_2_txt( Eigen::Matrix4f transf_IN , QString fileNameQT )
{
        std::ofstream streamWRITE(   fileNameQT.toStdString().data()   );
                      streamWRITE << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << transf_IN;
                      streamWRITE.close();
}



Eigen::Matrix4f Registrator::txt_2_transf( QString fileNameQT )
{
        Eigen::Matrix4f transfMat4f;

        std::ifstream                           streamREAD(   fileNameQT.toStdString().data()   );
        for     (int iii=0; iii<4; iii++)
            for (int jjj=0; jjj<4; jjj++)       streamREAD >> transfMat4f(iii,jjj);
                                                streamREAD.close();
        ///////////////////
        return transfMat4f;
        ///////////////////
}




