// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef ROBUSTMATCHER_H
#define ROBUSTMATCHER_H

#include <QElapsedTimer>
#include <QDir>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// RobustMatcher class taken from OpenCV2 Computer Vision Application Programming Cookbook Ch 9 //
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
class RobustMatcher
{
      private:

         cv::Ptr< cv::FeatureDetector     > detector;   // pointer to the feature point detector object
         cv::Ptr< cv::DescriptorExtractor > extractor;  // pointer to the feature descriptor extractor object
         cv::Ptr< cv::DescriptorMatcher   > matcher;    // pointer to the matcher object

         float  ratio;      // max ratio between 1st and 2nd NN
         bool   refineF;    // if true will refine the F matrix
         double distance;   // min distance to epipolar
         double confidence; // confidence level (probability)

         int fileNamePADDING;

      public:

         RobustMatcher() : ratio(0.65f), refineF(true), confidence(0.99), distance(3.0), fileNamePADDING(3)
         {
                detector  = new cv::SiftFeatureDetector();
                extractor = new cv::SiftDescriptorExtractor();
                matcher   = new cv::BFMatcher( cv::NORM_L2, false );                // http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#bfmatcher-bfmatcher
                                                                                    // NORM_L1 and NORM_L2 norms are preferable choices for SIFT and SURF descriptors
                                                                                    // NORM_HAMMING should be used with ORB, BRISK and BRIEF,
                                                                                    // NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4
                                                                                    // (see ORB::ORB constructor description)
                                                                                    //
                                                                                    // If crossCheck==true, then the knnMatch() method with k=1 will only return pairs (i,j) such that
                                                                                    // for i-th query descriptor the j-th descriptor in the matcher’s collection is the nearest and vice versa,
                                                                                    // i.e. the BFMatcher will only return consistent pairs. Such technique usually produces best results with minimal
                                                                                    // number of outliers when there are enough matches. This is alternative to the ratio test, used by D. Lowe in SIFT paper.
         }


         void setPARAMs( const int    &PARAM_SIFT_KEY_nFeatures,
                         const int    &PARAM_SIFT_KEY_nOctaveLayers,
                         const double &PARAM_SIFT_KEY_contrastThreshold,
                         const double &PARAM_SIFT_KEY_edgeThreshold,
                         const double &PARAM_SIFT_KEY_sigma,
                         const int    &fileNamePADDING_IN)
         {

             fileNamePADDING = fileNamePADDING_IN;

             /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             //std::vector<std::string> paramNamesKEY;
             //detector->getParams(     paramNamesKEY );
             //for (int nnn=0;      nnn<paramNamesKEY.size(); nnn++)
             //    std::cout << "SIFT_KEY_PARAMS - " << paramNamesKEY[nnn] << std::endl;
             /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             detector->set("nFeatures"        ,PARAM_SIFT_KEY_nFeatures);           // int
             detector->set("nOctaveLayers"    ,PARAM_SIFT_KEY_nOctaveLayers);       // int
             detector->set("contrastThreshold",PARAM_SIFT_KEY_contrastThreshold);   // double
             detector->set("edgeThreshold"    ,PARAM_SIFT_KEY_edgeThreshold);       // double
             detector->set("sigma"            ,PARAM_SIFT_KEY_sigma);               // double
             /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             //std::cout << "PARAM_SIFT_KEY_nFeatures        " << "  \t" << detector->getInt(   "nFeatures"        ) << std::endl;
             //std::cout << "PARAM_SIFT_KEY_nOctaveLayers    " << "  \t" << detector->getInt(   "nOctaveLayers"    ) << std::endl;
             //std::cout << "PARAM_SIFT_KEY_contrastThreshold" << "  \t" << detector->getDouble("contrastThreshold") << std::endl;
             //std::cout << "PARAM_SIFT_KEY_edgeThreshold    " << "  \t" << detector->getDouble("edgeThreshold"    ) << std::endl;
             //std::cout << "PARAM_SIFT_KEY_sigma            " << "  \t" << detector->getDouble("sigma"            ) << std::endl;
             /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


             ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             //std::vector<std::string> paramNamesFEAT;
             //extractor->getParams(    paramNamesFEAT);
             //for (int nnn=0;      nnn<paramNamesFEAT.size(); nnn++)
             //    std::cout << "SIFT_FEAT_PARAMS - " << paramNamesFEAT[nnn] << std::endl;
             ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             extractor->set("nFeatures"        ,PARAM_SIFT_KEY_nFeatures);           // int
             extractor->set("nOctaveLayers"    ,PARAM_SIFT_KEY_nOctaveLayers);       // int
             extractor->set("contrastThreshold",PARAM_SIFT_KEY_contrastThreshold);   // double
             extractor->set("edgeThreshold"    ,PARAM_SIFT_KEY_edgeThreshold);       // double
             extractor->set("sigma"            ,PARAM_SIFT_KEY_sigma);               // double
             ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             //std::cout << "PARAM_SIFT_FEAT_nFeatures        " << "  \t" << extractor->getInt(   "nFeatures"        ) << std::endl;
             //std::cout << "PARAM_SIFT_FEAT_nOctaveLayers    " << "  \t" << extractor->getInt(   "nOctaveLayers"    ) << std::endl;
             //std::cout << "PARAM_SIFT_FEAT_contrastThreshold" << "  \t" << extractor->getDouble("contrastThreshold") << std::endl;
             //std::cout << "PARAM_SIFT_FEAT_edgeThreshold    " << "  \t" << extractor->getDouble("edgeThreshold"    ) << std::endl;
             //std::cout << "PARAM_SIFT_FEAT_sigma            " << "  \t" << extractor->getDouble("sigma"            ) << std::endl;
             ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

         }


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // Set the feature detector
      void setFeatureDetector(       cv::Ptr<cv::FeatureDetector>&     detect)   {   detector   = detect;   }
      void setDescriptorExtractor(   cv::Ptr<cv::DescriptorExtractor>& desc  )   {   extractor  = desc;     }   // Set the descriptor extractor
      void setDescriptorMatcher(     cv::Ptr<cv::DescriptorMatcher>&   match )   {   matcher    = match;    }   // Set the matcher
      void setConfidenceLevel(       double conf )                               {   confidence = conf;     }   // Set confidence level
      void setMinDistanceToEpipolar( double dist )                               {   distance   = dist;     }   // Set MinDistanceToEpipolar
      void setRatio(                 float  rat  )                               {   ratio      = rat;      }   // Set ratio

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      // Clear matches for which NN ratio is > than threshold
      // return the number of removed points
      // (corresponding entries being cleared,
      // i.e. size will be 0)


      int ratioTest( std::vector<std::vector<cv::DMatch> > &matches)
      {

                int removed=0;
                  // for all matches
                for (std::vector<std::vector<cv::DMatch> >::iterator
                         matchIterator= matches.begin();
                     matchIterator!= matches.end(); ++matchIterator) {
                       // if 2 NN has been identified
                       if (matchIterator->size() > 1) {
                           // check distance ratio
                           if ((*matchIterator)[0].distance/
                               (*matchIterator)[1].distance > ratio) {
                              matchIterator->clear(); // remove match
                              removed++;
                           }
                       } else { // does not have 2 neighbours
                           matchIterator->clear(); // remove match
                           removed++;
                       }
                }
                return removed;

      } // ratioTest


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      // Insert symmetrical matches in symMatches vector


      void symmetryTest(    const std::vector<std::vector<cv::DMatch> > &matches1,
                            const std::vector<std::vector<cv::DMatch> > &matches2,
                                  std::vector<cv::DMatch>               &symMatches)
      {
            // for all matches image 1 -> image 2
            for (std::vector<std::vector<cv::DMatch> >::
                     const_iterator matchIterator1= matches1.begin();
                 matchIterator1!= matches1.end(); ++matchIterator1)
            {
                   // ignore deleted matches
                   if (matchIterator1->size() < 2)
                       continue;
                   // for all matches image 2 -> image 1
                   for (std::vector<std::vector<cv::DMatch> >::
                      const_iterator matchIterator2= matches2.begin();
                       matchIterator2!= matches2.end();
                       ++matchIterator2) {
                       // ignore deleted matches
                       if (matchIterator2->size() < 2)
                          continue;
                       // Match symmetry test
                       if ((*matchIterator1)[0].queryIdx ==
                           (*matchIterator2)[0].trainIdx &&
                           (*matchIterator2)[0].queryIdx ==
                           (*matchIterator1)[0].trainIdx) {
                           // add symmetrical match
                             symMatches.push_back(
                               cv::DMatch((*matchIterator1)[0].queryIdx,
                                         (*matchIterator1)[0].trainIdx,
                                         (*matchIterator1)[0].distance));
                             break; // next match in image 1 -> image 2
                       }
                   }
            }


      } // symmetryTest


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      // Identify good matches using RANSAC
      // Return fundemental matrix


      cv::Mat ransacTest(   const std::vector<cv::DMatch>     &matches,
                            const std::vector<cv::KeyPoint>   &keypoints1,
                            const std::vector<cv::KeyPoint>   &keypoints2,
                                  std::vector<cv::DMatch>     &outMatches)
      {
               // Convert keypoints into Point2f
               std::vector<cv::Point2f> points1, points2;
               cv::Mat fundemental;
               for (std::vector<cv::DMatch>::
                     const_iterator it= matches.begin();
                   it!= matches.end(); ++it)
               {
                       // Get the position of left keypoints
                       float x= keypoints1[it->queryIdx].pt.x;
                       float y= keypoints1[it->queryIdx].pt.y;
                       points1.push_back(cv::Point2f(x,y));
                       // Get the position of right keypoints
                       x= keypoints2[it->trainIdx].pt.x;
                       y= keypoints2[it->trainIdx].pt.y;
                       points2.push_back(cv::Point2f(x,y));
                }
               // Compute F matrix using RANSAC
               std::vector<uchar> inliers(points1.size(),0);
               if (points1.size()>0&&points2.size()>0)
               {
                      cv::Mat fundemental= cv::findFundamentalMat(      cv::Mat(points1),
                                                                        cv::Mat(points2),   // matching points
                                                                        inliers,            // match status (inlier or outlier)
                                                                        CV_FM_RANSAC,       // better
                                                                      //CV_FM_LMEDS,        // worse
                                                                        distance,           // distance to epipolar line
                                                                        confidence  );      // confidence probability
                      // extract the surviving (inliers) matches
                      std::vector<uchar>::const_iterator
                                         itIn= inliers.begin();
                      std::vector<cv::DMatch>::const_iterator
                                         itM= matches.begin();
                       // for all matches
                       for ( ;itIn!= inliers.end(); ++itIn, ++itM)
                       {
                          if (*itIn) { // it is a valid match
                             outMatches.push_back(*itM);
                          }
                       }
                       if (refineF)
                       {
                           // The F matrix will be recomputed with
                           // all accepted matches
                              // Convert keypoints into Point2f
                              // for final F computation
                              points1.clear();
                              points2.clear();
                              for (std::vector<cv::DMatch>::
                                     const_iterator it= outMatches.begin();
                                  it!= outMatches.end(); ++it)
                              {
                                      // Get the position of left keypoints
                                      float x= keypoints1[it->queryIdx].pt.x;
                                      float y= keypoints1[it->queryIdx].pt.y;
                                      points1.push_back(cv::Point2f(x,y));
                                      // Get the position of right keypoints
                                      x= keypoints2[it->trainIdx].pt.x;
                                      y= keypoints2[it->trainIdx].pt.y;
                                      points2.push_back(cv::Point2f(x,y));
                              }
                              // Compute 8-point F from all accepted matches
                              if (points1.size()>0&&points2.size()>0)
                              {
                                     fundemental= cv::findFundamentalMat(
                                        cv::Mat(points1),cv::Mat(points2), // matches
                                        CV_FM_8POINT); // 8-point method
                              }
                       }
                }
                return fundemental;


      } // ransacTest


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      // Match feature points using symmetry test and RANSAC
      // returns fundemental matrix
      /*cv::Mat*/void match( const  cv::Mat                     &image1,
                             const  cv::Mat                     &image2,
                             const  cv::Mat                     &mask1,
                             const  cv::Mat                     &mask2,
                                    std::vector<cv::DMatch>     &matches,
                                    std::vector<cv::KeyPoint>   &keypoints1,
                                    std::vector<cv::KeyPoint>   &keypoints2,
                             const  QString                     &caller_local_OR_global,
                             const  bool                        &FLAG_PERFORM_2D_FILTERING_TESTS)
      {


                matches.clear();
                keypoints1.clear();
                keypoints2.clear();


                cv::Mat descriptors1, descriptors2;


                if      (caller_local_OR_global == "local")
                {

                                                    detector->detect(   image1, keypoints1, mask1        ); // 1a. Detection of the SURF features
                                                    detector->detect(   image2, keypoints2, mask2        );

                        if (keypoints1.size()>0)    extractor->compute( image1, keypoints1, descriptors1 ); // 1b. Extraction of the SURF descriptors
                        if (keypoints2.size()>0)    extractor->compute( image2, keypoints2, descriptors2 );


                        //////////////////////////////
                        // or detector.write( fs ); //
                        //////////////////////////////

                        // SOURCE - 1
                        // TARGET - 2

                }


                if (descriptors1.rows==0 || descriptors1.cols==0 ||
                    descriptors2.rows==0 || descriptors2.cols==0  )
                {
                    return;
                }


                std::vector<std::vector<cv::DMatch> > matches1;
                std::vector<std::vector<cv::DMatch> > matches2;
               {
               // 2. Match the two image descriptors
               // Construction of the matcher
               // from image 1 to image 2
               // based on k nearest neighbours (with k=2)
               matcher->knnMatch( descriptors1,
                                  descriptors2,
                                  matches1,  // vector of matches (up to 2 per entry)
                                  2);        // return 2 nearest neighbours
                // from image 2 to image 1
                // based on k nearest neighbours (with k=2)
                matcher->knnMatch( descriptors2,
                                   descriptors1,
                                   matches2, // vector of matches (up to 2 per entry)
                                   2);       // return 2 nearest neighbours
                }


                {
                // 3. Remove matches for which NN ratio is
                // > than threshold
                // clean image 1 -> image 2 matches
                int removed= ratioTest( matches1 );
                // clean image 2 -> image 1 matches
                    removed= ratioTest( matches2 );
                }


                std::vector<cv::DMatch> symMatches;
                cv::Mat                 fundemental;


                {
                // 4. Remove non-symmetrical matches
                symmetryTest( matches1, matches2, symMatches );
                }


                if (FLAG_PERFORM_2D_FILTERING_TESTS)
                {
                // 5. Validate matches using RANSAC
                fundemental = ransacTest( symMatches,
                                          keypoints1,
                                          keypoints2,
                                          matches );
                // return the found fundemental matrix
                }
                else
                {
                    matches = symMatches;
                }


      }


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


};

#endif // ROBUSTMATCHER_H





