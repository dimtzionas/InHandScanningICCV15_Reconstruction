// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>




void Registrator::my_Feature_Descriptor_on_ISS3D( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &searchSurface_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &inputNormals_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloud_KEYp__3D,
                                                  double                                  &model_resolution,
                                                  pcl::PointCloud<TYPE_feature>::Ptr      &FEAT__3D__cloud_OUT     )
{

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        pcl::SHOTColorEstimationOMP< TYPE_Point_PostPr,TYPE_Point_PostPr,TYPE_feature > featureDescriptor; // float descriptor [1344] || float rf [9]   // ISS3D
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        pcl::search::KdTree<TYPE_Point_PostPr>::Ptr tree (new pcl::search::KdTree<TYPE_Point_PostPr>);
        featureDescriptor.setSearchSurface( searchSurface_cloud_UNorg_PN );
        featureDescriptor.setInputNormals(  inputNormals_cloud_UNorg_PN );
        featureDescriptor.setInputCloud(    cloud_KEYp__3D );
        featureDescriptor.setSearchMethod(  tree );
        featureDescriptor.setRadiusSearch(  PARAM_FEAT_coeffRadiusSearch * model_resolution ); // IMPORTANT: must be larger than radius used to estimate normals!!!
        featureDescriptor.compute(         *FEAT__3D__cloud_OUT );

        ///////////////////////////////
        bool deletedAtLeastOne = false;
        ///////////////////////////////


        ///////////////////////////////////////////////////////////////////////////////////////
        int descriptorSize = sizeof((*FEAT__3D__cloud_OUT)[0].descriptor) / (int)sizeof(float);
        int         rfSize = sizeof((*FEAT__3D__cloud_OUT)[0].rf)         / (int)sizeof(float);
        ///////////////////////////////////////////////////////////////////////////////////////


        if (FEAT__3D__cloud_OUT->is_dense == false)
        {

                for (int fff=FEAT__3D__cloud_OUT->size()-1; fff>=0; fff--)
                {

                        ////////////////////////////
                        bool deletedAlready = false;
                        ////////////////////////////

                        for (int ddd=0; ddd<descriptorSize; ddd++)
                        {
                                if (   pcl_isfinite( (*FEAT__3D__cloud_OUT)[fff].descriptor[ddd] ) == false   ) // A number is finite if it is neither NaN (Not a Number) nor positive or negative infinity.
                                {
                                    (*FEAT__3D__cloud_OUT).erase(  (*FEAT__3D__cloud_OUT).begin() + fff  );
                                    (*cloud_KEYp__3D)     .erase(  (*cloud_KEYp__3D)     .begin() + fff  );

                                    std::cout << "---- Deleted NAN Feature + KeyPoint - " << fff << std::endl;

                                    deletedAlready = true;
                                    deletedAtLeastOne = true;
                                    break;
                                }
                        }

                        /////////////////////////////////////////
                        if (deletedAlready == true)     continue;
                        /////////////////////////////////////////

                        for (int rrr=0; rrr<rfSize; rrr++)
                        {
                                if (   pcl_isfinite( (*FEAT__3D__cloud_OUT)[fff].rf[rrr] ) == false   ) // A number is finite if it is neither NaN (Not a Number) nor positive or negative infinity.
                                {
                                    (*FEAT__3D__cloud_OUT).erase(  (*FEAT__3D__cloud_OUT).begin() + fff  );
                                    (*cloud_KEYp__3D)     .erase(  (*cloud_KEYp__3D)     .begin() + fff  );

                                    std::cout << "---- Deleted NAN Feature + KeyPoint - " << fff << std::endl;

                                    deletedAtLeastOne = true;
                                    break;
                                }
                        }

                }

        }

}
