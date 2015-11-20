// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>





void Registrator::BackProject_Depth_2_PCL( const cv::Mat                            &Img_COL_RAWW_Curr,
                                           const cv::Mat                            &Img_DDD_RAWW_Curr,
                                                 pcl::PointCloud<pcl::PointXYZRGB>  &cloud_ORGan_P_,
                                           const int                                 morphSIZ,
                                           const bool                                should_FLIP_Y,
                                           const bool                                kickOut_SkinColored )
{

        /////////////////////////////////
        int www = Img_COL_RAWW_Curr.cols;
        int hhh = Img_COL_RAWW_Curr.rows;
        /////////////////////////////////


        ////////////////////////////////////
        double fx = PARAM_CAM_Intrinsics_fx;
        double fy = PARAM_CAM_Intrinsics_fy;
        double cx = PARAM_CAM_Intrinsics_px;
        double cy = PARAM_CAM_Intrinsics_py;
        ////////////////////////////////////


        //////////////////////////////
        cloud_ORGan_P_.points.clear();
        //////////////////////////////


        ////////////////////////////////////////////////
        //// Ensure Float Depth image in mm ////////////
        ////////////////////////////////////////////////
        cv::Mat                      DEPTH;
        Img_DDD_RAWW_Curr.convertTo( DEPTH, CV_32FC1 );
        ////////////////////////////////////////////////
        ////////////////////////////////////////////////
        ////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        cv::Mat MASK = cv::Mat::zeros(Img_DDD_RAWW_Curr.rows,Img_DDD_RAWW_Curr.cols,CV_8UC1);
        cv::threshold( Img_DDD_RAWW_Curr, MASK, 0, 255, cv::THRESH_BINARY );
        MASK.convertTo( MASK, CV_8UC1 ); // OK
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (morphSIZ > 0)
        {
            cv::erode(  MASK, MASK, cv::getStructuringElement( cv::MORPH_RECT, cv::Size2i(morphSIZ,morphSIZ) ) );
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        cv::Mat                    skinnBIN = cv::Mat::zeros( Img_COL_RAWW_Curr.rows, Img_COL_RAWW_Curr.cols, CV_8UC1 );
        if (kickOut_SkinColored)   skinnBIN = skinnDetector.compute_SkinnBinMap_4_Image( Img_COL_RAWW_Curr, Img_DDD_RAWW_Curr, PARAM_SkinnDetector_THRESH, true, false, false );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        for     (int iii=0; iii<hhh; iii++)
        {   for (int jjj=0; jjj<www; jjj++)
            {
                double  currDepth =      DEPTH.at<float>(iii,jjj);
                int     msk       = (int)MASK. at<uchar>(iii,jjj);
                bool    isSkin    = (skinnBIN. at<uchar>(iii,jjj)==255);


                PCD_RGB currColor;
                        currColor.R = Img_COL_RAWW_Curr.at<cv::Vec3b>(iii,jjj)[0];
                        currColor.G = Img_COL_RAWW_Curr.at<cv::Vec3b>(iii,jjj)[1];
                        currColor.B = Img_COL_RAWW_Curr.at<cv::Vec3b>(iii,jjj)[2];

                pcl::PointXYZRGB      currVertex;


                if (msk == 0 || isSkin)     {       currVertex.x = std::numeric_limits<float>::quiet_NaN();
                                                    currVertex.y = std::numeric_limits<float>::quiet_NaN();
                                                    currVertex.z = std::numeric_limits<float>::quiet_NaN();
                                            }
                else                        {       currVertex.x = (jjj - cx) * currDepth / fx;                              ///////////////////
                                                    currVertex.y = (iii - cy) * currDepth / fy;          if (should_FLIP_Y)  currVertex.y *= -1;
                                                    currVertex.z =              currDepth;                                   ///////////////////

                                                    if (currVertex.z == 0)
                                                    {
                                                        currVertex.z = std::numeric_limits<float>::quiet_NaN();
                                                        std::cout << "\n\n" << "MPAM nan thingy" << "\n" << std::endl;
                                                        exit(1);
                                                    }
                                            }
                                                    currVertex.rgb = currColor.RGB_float;

                //////////////////////////////////////////////
                cloud_ORGan_P_.points.push_back( currVertex );
                //////////////////////////////////////////////

            }
        }


        ////////////////////////////
        cloud_ORGan_P_.width  = www;
        cloud_ORGan_P_.height = hhh;
        ////////////////////////////////
        cloud_ORGan_P_.is_dense = false;
        ////////////////////////////////


}

