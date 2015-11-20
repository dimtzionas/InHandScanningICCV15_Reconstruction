// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include <QCoreApplication>

#include <QDateTime>

#include <QElapsedTimer>
#include <QDir>
#include <QVector>

#include <pcl/common/transformation_from_correspondences.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>

#include <pcl/io/ply_io.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <pcl/PolygonMesh.h>

#include <boost/thread/thread.hpp>

#include <pcl/common/common.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree.h>
#include <ctime>

#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/tsdf_volume_octree.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>

#include <Eigen/LU>
#include <Eigen/Geometry>

#include <touch.h>
#include <renderer.h>

#include <libconfig.h++>

#include <opencv2/opencv.hpp>

#include <skinndetector.h>

#include <robustMatcher.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


union PCD_RGB
{
        struct
        {
            uchar R; // LSB
            uchar G; // ---
            uchar B; // MSB
        };
        float RGB_float;
        uint  RGB_uint;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct TextureCORR
{
    Eigen::Vector3f         source_3D_pt_3f;
    Eigen::Vector3f         target_3D_pt_3f;
    Eigen::Vector3f         source_3D_nrm3f;
    Eigen::Vector3f         target_3D_nrm3f;
    pcl::PointXYZRGBNormal  source_3D_ptXYZRGBNormal;
    pcl::PointXYZRGBNormal  target_3D_ptXYZRGBNormal;
    int                     source_iii_atORGpcl;
    int                     source_jjj_atORGpcl;
    int                     target_iii_atORGpcl;
    int                     target_jjj_atORGpcl;
};

struct GeometrCORR
{
    Eigen::Vector3f         source_3D_pt_3f;
    Eigen::Vector3f         target_3D_pt_3f;
    Eigen::Vector3f         source_3D_nrm3f;
    Eigen::Vector3f         target_3D_nrm3f;
    pcl::PointXYZRGBNormal  source_3D_ptXYZRGBNormal;
    pcl::PointXYZRGBNormal  target_3D_ptXYZRGBNormal;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct FUSED_CORRR_label
{
    int     label_ID;
    QString label_Str;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct PoseNotebookElement
{
    int             sourceID_listID;
    int             targetID_listID;
    Eigen::Matrix4f relativeTransf;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



class Registrator
{
public:

    //////////////////////////////////////////////////
    //////////////////////////////////////////////////
    typedef pcl::PointXYZRGB        TYPE_Point_Sensor;
    typedef pcl::PointXYZRGBNormal  TYPE_Point_PostPr;
    //////////////////////////////////////////////////
    typedef TYPE_Point_PostPr       TYPE_Point_KEYpt;
    //////////////////////////////////////////////////
    typedef pcl::SHOT1344           TYPE_feature;
    //////////////////////////////////////////////////
    //////////////////////////////////////////////////

    void my_KeyPoint_Detector_ISS3D(  const pcl::PointCloud<TYPE_Point_PostPr>::Ptr   &cloudIN_PN,
                                            double                                    &modelResolutionIN,
                                            pcl::PointCloud<TYPE_Point_PostPr>::Ptr   &keyPointCloudOUT_PN    );

    void my_Feature_Descriptor_on_ISS3D(   pcl::PointCloud<TYPE_Point_PostPr  >::Ptr  &searchSurface_cloud_UNorg_PN,
                                           pcl::PointCloud<TYPE_Point_PostPr  >::Ptr  &inputNormals_cloud_UNorg_PN,
                                           pcl::PointCloud<TYPE_Point_PostPr  >::Ptr  &cloud_KEYp__3D,
                                           double                                     &model_resolution,
                                           pcl::PointCloud<TYPE_feature       >::Ptr  &target_cloud_FEAT__3D_OUT   );


    Registrator( QString fileName_CONFIG_RUN, QString fileName_CONFIG_PARAMs,                                                                                                                                   bool FLAG_touchFrames_PRINT, bool FLAG_print_CONFIG );
    Registrator(    bool applyHANDs_IN, bool hasHANDs_IN, QString INPUT_PATH, QString RunningMODE, bool VIEW_ENABLED, bool FEAT_ENABLED, bool ICP_ENABLED, QString syntheticORrealistic_IN, QString commentStr, bool FLAG_touchFrames_PRINT, bool FLAG_print_CONFIG );
    void construct( bool applyHANDs_IN, bool hasHANDs_IN, QString INPUT_PATH, QString RunningMODE, bool VIEW_ENABLED, bool FEAT_ENABLED, bool ICP_ENABLED, QString syntheticORrealistic_IN, QString commentStr, bool FLAG_touchFrames_PRINT, bool FLAG_print_CONFIG );

    RobustMatcher rmatcher;
    double PARAM_RMATCHER_Ratio;
    double PARAM_RMATCHER_MinDistanceToEpipolar;
    double PARAM_RMATCHER_ConfidenceLevel;

    bool applyHANDs;
    bool   hasHANDs;


    Sequence* sequence;

    Touch* touch;

    Renderer* renderer;


    void resolution_bilateral_normals_outlierRemoval_passthrough_KEEP_ORG(      pcl::PointCloud<TYPE_Point_Sensor>::Ptr &source_cloud_ORGan_P_,
                                                                                pcl::PointCloud<TYPE_Point_Sensor>::Ptr &target_cloud_ORGan_P_,
                                                                                pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                                                pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                                                double                                  &model_resolution_Source,
                                                                                double                                  &model_resolution_Target,
                                                                                const bool                              &TEST_MODE
                                                                         );
    void resolution_bilateral_normals_outlierRemoval_passthrough_KEEP_ORG(      pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_,
                                                                                pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloud_UNorg_PN,
                                                                                double                                  &model_resolution,
                                                                                const bool                              &TEST_MODE
                                                                         );


    Eigen::Matrix4f myTransformationEstimationPointToPointWeighted( const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr &source_cloud_KEY,
                                                                    const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr &target_cloud_KEY,
                                                                    const pcl::CorrespondencesPtr                 &corr_Filt );

    int    PARAM_SIFT_KEY_nFeatures;
    int    PARAM_SIFT_KEY_nOctaveLayers;
    double PARAM_SIFT_KEY_contrastThreshold;
    double PARAM_SIFT_KEY_edgeThreshold;
    double PARAM_SIFT_KEY_sigma;

    void corresp_2_pt_source( const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr &cloud_PN, const pcl::CorrespondencesPtr &corrSet, const int index, Eigen::Vector3f &ptVec3f );
    void corresp_2_pt_target( const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr &cloud_PN, const pcl::CorrespondencesPtr &corrSet, const int index, Eigen::Vector3f &ptVec3f );

    SkinnDetector skinnDetector;

    void            transf_2_txt( Eigen::Matrix4f transf_IN , QString fileNameQT );
    Eigen::Matrix4f txt_2_transf(                             QString fileNameQT );


    double computeCloudResolution(const pcl::PointCloud<TYPE_Point_PostPr>::ConstPtr &cloud);
    double computeCloudResolution(const pcl::PointCloud<TYPE_Point_Sensor>::ConstPtr &cloud);


    void     read_CONFIG_PARAMs();
    void    print_CONFIG_PARAMs();
    void     read_CONFIG_PATHs( int seqID );
    void    print_CONFIG_PATHs();
    void     read_CONFIG_DBG();
    void    print_CONFIG_DBG();
    void     read_CONFIG_RUN();
    void    print_CONFIG_RUN();
    QString  PATH_CONFIG_baseInput;
    QString  PATH_CONFIG_RUN;
    QString  PATH_CONFIG_PARAMs;
    QString  PATH_CONFIG_PATHs;
    QString  PATH_CONFIG_DBG;


    QVector<PoseNotebookElement> poseNoteBook;


    void refineTextureCORRs_withPostProccessedPCL( const pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_POSTT_PN,
                                                   const pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_POSTT_PN,
                                                         QVector<TextureCORR>                    &myTextureCORR,
                                                   const bool                                    &TEST_MODE,
                                                   const QString                                 &p2p_p2plane );


    QString getOnly_fileNameWithExtension( QString FULL_PATH );

    void texture3Dpoints_TRANSFORM_Source( Eigen::Matrix4f transformationMatrix4f );
    void texture3Dpoints_TRANSFORM_Target( Eigen::Matrix4f transformationMatrix4f );

    QString PATH_MODELS_INFO;
    QString PATH_INDEX_BOUNDS;
    QString PATH_CAMERA_SET;

    QString PATH_OUTPUT_BASE;

    double PARAM_NORMAL_MaxDepthChangeFactor;
    double PARAM_NORMAL_NormalSmoothingSize;
    double PARAM_NORMAL_NormalSmoothingSizeCOEFF;
    double PARAM_DBG_Normals_VizDistCOEFF;
    double PARAM_DBG_Normals_VizSizeCOEFF;
    double PARAM_KEYP_coeffSalientRadius;
    double PARAM_KEYP_coeffNonMaxRadiuss;
    double PARAM_FEAT_coeffRadiusSearch;
    bool   PARAM_CORR_REJ_Kick_1_to_1;
    double PARAM_CORR_REJ_InlierThreshold_LOCAL;
    double PARAM_ICP_RansacOutlierRejectionThreshold;
    double PARAM_ICP_RansacOutlierRej__TRANSITION;
    double PARAM_ICP_RansacOutlierRej__touchIDs;
    double PARAM_ICP_RansacOutlierRej_p2p_TRICK;
    double PARAM_ICP_TransformationEpsilon;
    double PARAM_ICP_EuclideanFitnessEpsilon;
    int    PARAM_ICP_MAX_ITER;

    bool   PARAM_CORR_FEAT2D_PerformRansac2D;

    QString PARAM_ICP_Mode__previous_OR_metascan;
    QString PARAM_ICP_Mode__p2p_OR_p2plane;
    QString PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION;
    bool    PARAM_ICP_Mode__p2p_TRICK;
    QString PARAM_OBJ_Mode__p2p_OR_p2plane;

    bool PARAM_KILL_BILATERAL;
    bool PARAM_KILL_SOR;
    bool PARAM_KILL_PASS;

    double PARAM_TSDF_GridSize;
    int    PARAM_TSDF_Resolution;
    bool   PARAM_TSDF_Color;
    bool   PARAM_TSDF_ColorSetByConf;
    double PARAM_TSDF_DepthTrunc_MaxPOS;
    double PARAM_TSDF_DepthTrunc_MaxNEG;
    double PARAM_TSDF_SensorDistBound_MIN;
    double PARAM_TSDF_SensorDistBound_MAX;
    double PARAM_CAM_Intrinsics_fx;
    double PARAM_CAM_Intrinsics_fy;
    double PARAM_CAM_Intrinsics_px;
    double PARAM_CAM_Intrinsics_py;
    double PARAM_TSDF_ISO_level;
    double PARAM_TSDF_PercExtendGrid;
    double PARAM_TSDF_Min_Weight_MC;
    int    PARAM_TSDF_Img_Width;
    int    PARAM_TSDF_Img_Height;
    double PARAM_TSDF_MaxWeightTruncLimit;
    int    PARAM_TSDF_NumRandomSplts;
    double PARAM_TSDF_MaxVoxelSize;

    int    PARAM_SOR_MeanK;
    double PARAM_SOR_StddevMulThresh;

    double PARAM_PASS_FILTER_XXX_min;
    double PARAM_PASS_FILTER_XXX_max;
    double PARAM_PASS_FILTER_YYY_min;
    double PARAM_PASS_FILTER_YYY_max;
    double PARAM_PASS_FILTER_ZZZ_min;
    double PARAM_PASS_FILTER_ZZZ_max;
    double PARAM_PASS_FILTER_Sph_SIZ;


    void restore_NANs___dueToPCLbugInBILATERAL(           pcl::PointCloud<TYPE_Point_Sensor>::Ptr& cloud_ORGan_P_, bool shouldPrint_countYesN_NANs );
    void restore_NANs___dueToPCLbugInBILATERAL(           pcl::PointCloud<TYPE_Point_PostPr>::Ptr& cloud_ORGan_P_, bool shouldPrint_countYesN_NANs );

    void potentialUnitChange_PCL_m2mm( pcl::PointCloud<TYPE_Point_Sensor>::Ptr& cloud_ORGan_P_ );

    cv::Mat getMASK_denoised_fromPCL___handsYES( const pcl::PointCloud<TYPE_Point_Sensor>::Ptr  &cloud_ORGan_P_ );
    cv::Mat getMASK_denoised_fromPCL___handsNOO( const cv::Mat                                  &depthMap       );


    void readIMG_getPCL( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_,                                   const bool PRINT_Paths );
    void readIMG_getPCL( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_, cv::Mat &imgRGB, cv::Mat &imgDDD, const bool PRINT_Paths );
    void readIMG       ( const QString &INPUT_PATH, const int &fileID,                                                         cv::Mat &imgRGB, cv::Mat &imgDDD, const bool PRINT_Paths );
    void readPCL       ( const QString &INPUT_PATH, const int &fileID, pcl::PointCloud<TYPE_Point_Sensor>::Ptr &CURR_cloud_P_,                                   const bool PRINT_Paths );


    QVector<TextureCORR> corr_FEAT__2D;
    QVector<GeometrCORR> corr_FEAT__3D;


    void readIMGs_getPCLs_getFEAT2D(    const int                                       &sourceID_fileID,
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
                                        const bool                                      &PRINT_Paths  );

    void readIMGs_getPCLs_getFEAT2D(    const int                                       &fileID,
                                        const QString                                   &INPUT_PATH,
                                        const QString                                   &OUTPUT_PATH,
                                        const bool                                      &FEAT_ENABLED_2d,
                                        const int                                       &ifSTART_fileID,
                                        const QString                                   &caller_local_OR_global,
                                              pcl::PointCloud<TYPE_Point_Sensor  >::Ptr &cloud_ORGan_P_,
                                              QVector<TextureCORR>                      &myTextureCORR,
                                        const bool                                      &debugModeON_fromGlobal,
                                        const bool                                      &debug_1PAIR_fromGlobal,
                                        const bool                                      &PRINT_Paths  );

    void readIMGs_getPCLs_asRawAsPossible_kickedOutHandsWhenApplicable( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &CURR_cloud_ORGan_Pn,
                                                                        QString                                  INPUT_PATH         ,
                                                                        int                                      fileID             );

    void readIMGs_getFEAT2D(            const int                                       &sourceID_fileID,
                                        const int                                       &targetID_fileID,
                                        const QString                                   &INPUT_PATH,
                                        const QString                                   &OUTPUT_PATH,
                                        const bool                                      &FEAT_ENABLED_2d,
                                        const int                                       &ifSTART_fileID,
                                        const QString                                   &caller_local_OR_global,
                                              QVector<TextureCORR>                      &myTextureCORR,
                                        const bool                                      &debugModeON_fromGlobal,
                                        const bool                                      &debug_1PAIR_fromGlobal,
                                        const bool                                      &PRINT_Paths );



    double PARAM_VOXELFILTER_leafSize;
    double PARAM_VOXELFILTER_leafSizeCOEFF;
    double PARAM_VOXELFILTER_leafSize_FINAL_MESH;

    double PARAM_MORPH_ErodeSize;

    double PARAM_BILATERAL_SigmaS;
    double PARAM_BILATERAL_SigmaR;

    int    PARAM_LUM_MaxIterations;
    double PARAM_LUM_ConvergenceThreshold;
    int    PARAM_LLL_MinInliersNumber;
    int    PARAM_CORR_REJ_MaximumIterations;
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    bool PARAM_DBG_Apply_Transf_Obj_Feat2d;
    bool PARAM_DBG_Apply_Transf_Obj_Feat3d;
    bool PARAM_DBG_Apply_Transf_Skin;
    bool PARAM_DBG_Apply_Transf_ICP;

    bool PARAM_DBG_RenderObj_Source;
    bool PARAM_DBG_RenderObj_Target;

    bool PARAM_DBG_RenderObjNORMALS_Source;
    bool PARAM_DBG_RenderObjNORMALS_Target;

    bool PARAM_DBG_RenderSkeleton_Source;
    bool PARAM_DBG_RenderSkeleton_Target;

    bool PARAM_DBG_RenderMesh_Source;
    bool PARAM_DBG_RenderMesh_Target;

    bool PARAM_DBG_RenderPrimitive_Source;
    bool PARAM_DBG_RenderPrimitive_Target;

    bool PARAM_DBG_Render_PASS_FILTER;

    bool    PARAM_DBG_RenderCORR_Obj_Feat2d;
    bool    PARAM_DBG_RenderCORR_Obj_Feat3d;
    bool    PARAM_DBG_RenderCORR_Obj_FakeTransf;
    bool    PARAM_DBG_RenderCORR_Skin;
    int     PARAM_DBG_RenderCORR_SkinADD;

    bool   PARAM_DBG_RenderCORR_Skin_TouchVertSphere;
    double PARAM_DBG_RenderCORR_Skin_TouchVertSphereSiz;

    bool   PARAM_DBG_addCoordinateSystem;
    double PARAM_DBG_addCoordinateSysSIZ;

    int PARAM_DBG_offfff;
    int PARAM_DBG_offInter;

    double PARAM_DBG_offx;
    double PARAM_DBG_offy;
    double PARAM_DBG_offz;

    bool PARAM_DBG_cameraFlipX;
    bool PARAM_DBG_cameraFlipY;
    bool PARAM_DBG_cameraFlipZ;

    bool PARAM_DBG_RenderMesh_Source__Hands;
    bool PARAM_DBG_RenderMesh_Source__Object;
    bool PARAM_DBG_RenderMesh_Source__Normals;
    bool PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1;

    bool PARAM_DBG_RenderMesh_Target__Hands;
    bool PARAM_DBG_RenderMesh_Target__Object;
    bool PARAM_DBG_RenderMesh_Target__Normals;
    bool PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1;

    bool PARAM_DBG_Renderable_Model_Hand_R;
    bool PARAM_DBG_Renderable_Model_Hand_L;
    bool PARAM_DBG_Renderable_Model_Object;

    bool PARAM_DBG_OverloadPCL_Source_ALL;
    bool PARAM_DBG_OverloadPCL_Target_ALL;
    bool PARAM_DBG_OverloadPCL_Source_HandsNonOccl;
    bool PARAM_DBG_OverloadPCL_Target_HandsNonOccl;

    double PARAM_THRESH_CONV_m2mm;

    double PARAM_BackGround_RRR;
    double PARAM_BackGround_GGG;
    double PARAM_BackGround_BBB;

    int    PARAM_Viewer_PNG_Size_WWW;
    int    PARAM_Viewer_PNG_Size_HHH;

    void PARAM_DBG_flags__regulateActivation( bool TEST_MODE );
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    bool    PARAM_RUN_applyHANDs;
    bool    PARAM_RUN_hasHANDs;
    QString PARAM_RUN_INPUT_PATH;
    QString PARAM_RUN_RunningMODE;
    bool    PARAM_RUN_VIEW_ENABLED;
    bool    PARAM_RUN_FEAT_ENABLED;
    bool    PARAM_RUN_ICP_ENABLED;
    QString PARAM_RUN_syntheticORrealistic;
    QString PARAM_RUN_commentStr;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    bool    PARAM_RUN__ONLY__TOUCH__TRANSF;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    bool   PARAM_RUN_EXTRA_DBG_COUT;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    double PARAM_RUN_CorrWeight_OBJ_Feat2d;
    double PARAM_RUN_CorrWeight_OBJ_Feat3d;
    double PARAM_RUN_CorrWeight_SKIN;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    bool PARAM_DBG_OBJ_FEAT_2D_Source;
    bool PARAM_DBG_OBJ_FEAT_2D_Target;
    bool PARAM_DBG_OBJ_FEAT_2D_SHOW;
    bool PARAM_DBG_OBJ_FEAT_2D_FLIP;
    bool PARAM_DBG_OBJ_FEAT_3D_Source;
    bool PARAM_DBG_OBJ_FEAT_3D_Target;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    struct PARAM_INDEX_List_ELEMENT_Frame
    {
        int   listID; // ID in this vector
        int   fileID;
        int   pairID;
        bool isSTARTTT;
        bool isENDDDDD;
    };
    struct PARAM_INDEX_List_ELEMENT_IndexPair
    {
        int STARTTT_fileID;
        int ENDDDDD_fileID;

        Eigen::Vector3f dbg_centroidCheck_STARTTT_centr;
        Eigen::Vector3f dbg_centroidCheck_ENDDDDD_centr;
        int             dbg_centroidCheck_STARTTT_idddd;
        int             dbg_centroidCheck_ENDDDDD_idddd;
    };
    struct PARAM_INDEX_List_STRUCT
    {
        QVector<PARAM_INDEX_List_ELEMENT_Frame    >  frames;
        QVector<PARAM_INDEX_List_ELEMENT_IndexPair>  indexPairs;
    };
    PARAM_INDEX_List_STRUCT   PARAM_INDEX_List;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    double PARAM_KEYpt_min_scale;
    int    PARAM_KEYpt_nr_octaves;
    int    PARAM_KEYpt_nr_scales_per_octave;
    double PARAM_KEYpt_min_contrast;
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////

    int fileNamePADDING;

    void touchIndices_READ(  QString FULL_PATH );
    void touchIndices_PRINT();

    void touchFrames_from_touchIndices_COMPUTE();
    void touchFrames_PRINT();


    Eigen::Vector3f   myCentroid( const pcl::PointCloud< TYPE_Point_PostPr>::Ptr& cloud );
    Eigen::Vector3f   myCentroid( const pcl::PointCloud< TYPE_Point_Sensor>::Ptr& cloud );

    void my_Bilateral_Filtering( pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_, double resolution );
    void my_Bilateral_Filtering( pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_, double resolution, pcl::PointIndices &removedPointIndices );

    void my_Normal_Estimator(           const pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_,     // IN
                                              pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloud_UNorg_PN,     // OUT
                                        const QString                                 &syntheticORrealistic,
                                        const bool                                    &FLAG_kickOut_pointsWith_CoordNAN_NormalNAN );

    void kickOut_pointsWith_CoordNAN_NormalNAN(           pcl::PointCloud<TYPE_Point_PostPr>::Ptr    &cloud_PN );
    void kickOut_pointsWith_CoordNAN(                     pcl::PointCloud<TYPE_Point_Sensor>::Ptr    &cloud_PN );


    void my_Correspondence_Estimation(              const pcl::PointCloud<TYPE_feature>::Ptr         &source_cloud_FEAT__3D__IN,      // corrEst.setInputSource
                                                    const pcl::PointCloud<TYPE_feature>::Ptr         &target_cloud_FEAT__3D__IN,      // corrEst.setInputTarget
                                                    const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr    &source_cloud_KEYp__3D__IN,
                                                    const pcl::PointCloud<TYPE_Point_KEYpt >::Ptr    &target_cloud_KEYp__3D__IN,
                                                          QVector<GeometrCORR>                       &corr3D_Alll_OBJ_from3D);        // OUT
    void my_Correspondence_Filtering_n_Rejection(   const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr     &source_cloud_KEYp__3D__IN,      // corrRej.setInputSource
                                                    const pcl::PointCloud<TYPE_Point_KEYpt>::Ptr     &target_cloud_KEYp__3D__IN,      // corrRej.setInputTarget
                                                    const double                                     &PARAM_CORR_REJ_InlierThreshold, // IN
                                                    const bool                                       &PARAM_CORR_REJ_Kick_1_to_1,     // IN
                                                          pcl::CorrespondencesPtr                    &corrFUSED_Filt);                // OUT




    Eigen::Matrix4f My_ICP_Refinement___p2planeW( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER );
    Eigen::Matrix4f My_ICP_Refinement___p2planeW( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER,
                                                  Eigen::Matrix4f                          guess);

    Eigen::Matrix4f My_ICP_Refinement___p2plane(  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER );
    Eigen::Matrix4f My_ICP_Refinement___p2plane(  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER,
                                                  Eigen::Matrix4f                          guess);

    Eigen::Matrix4f My_ICP_Refinement___p2p(      pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER );
    Eigen::Matrix4f My_ICP_Refinement___p2p(      pcl::PointCloud<TYPE_Point_PostPr>::Ptr &source_cloud_UNorg_PN,
                                                  pcl::PointCloud<TYPE_Point_PostPr>::Ptr &target_cloud_UNorg_PN,
                                                  double                                   PARAM_RansacOutlierRejectionThreshold,
                                                  double                                   PARAM_TransformationEpsilon,
                                                  double                                   PARAM_EuclideanFitnessEpsilon,
                                                  int                                      PARAM_MAX_ITER,
                                                  Eigen::Matrix4f                         &guess);

    void my_Filtering_Voxelgrid( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &accumAlignedClouds_UNorg_P_ );

    Eigen::Matrix4f getRelativeTransformation(                                 const Eigen::Matrix4f &pose_iii,
                                                                               const Eigen::Matrix4f &pose_iii_m1 );
    void            getRelative__diff_Angle__diff_Trans(                       const Eigen::Matrix4f &Pose_target,
                                                                               const Eigen::Matrix4f &Pose_source,
                                                                                     double           &diff_Angle,
                                                                                     double          &diff_Trans);
    void            getRelative__diff_Angle__diff_Trans__relativeTransf(       const Eigen::Matrix4f &Pose_target,
                                                                               const Eigen::Matrix4f &Pose_source,
                                                                                     double          &diff_Angle,
                                                                                     double          &diff_Trans,
                                                                                     Eigen::Matrix4f &currPoseREL);
    void            getRelative__diff_Angle__diff_Trans__FROM__relativeTransf( const Eigen::Matrix4f &currPoseREL,
                                                                                     double          &diff_Angle,
                                                                                     double          &diff_Trans);

    void FUSE_CORRESPONDENCES(       pcl::PointCloud<TYPE_Point_KEYpt>::Ptr    &source_cloud_KEYfused,
                                     pcl::PointCloud<TYPE_Point_KEYpt>::Ptr    &target_cloud_KEYfused,
                                     pcl::CorrespondencesPtr                   &corrFUSED_Filt,
                                     QVector<FUSED_CORRR_label>                &corrFUSED_NonFilt_labels,
                               const double                                    &IN_PARAM_RUN_CorrWeight_OBJ_Feat2d,
                               const double                                    &IN_PARAM_RUN_CorrWeight_OBJ_Feat3d,
                               const double                                    &IN_PARAM_RUN_CorrWeight_SKIN,
                               const bool                                      &TEST_MODE,
                               int                                             &KEYfusedID,
                               const bool                                      &PARAM_RUN__ONLY__TOUCH__TRANSF_IN     );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void myReg_1_LocalAlign(    bool TEST_MODE, QString INPUT_PATH, QString OUTPUT_PATH, int NUMBBB, bool VIEW_ENABLED, bool FEAT_ENABLED, bool ICP_ENABLED, QString input__IMG__PCL,  int offHand );
    void myReg_X_TSDF_local(                    QString INPUT_PATH, QString OUTPUT_PATH, int NUMBBB, bool VIEW_ENABLED,                                      QString input__IMG__PCL, bool shouldPerformTSDF );
    void myReg_X_MESHLAB_local(                                     QString OUTPUT_PATH);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void myLoadCameraParams( boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer );
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    void BackProject_Depth_2_PCL( const cv::Mat                      &Img_COL_RAWW_Curr,
                                  const cv::Mat                      &Img_DDD_RAWW_Curr,
                                  pcl::PointCloud<pcl::PointXYZRGB>  &cloud_ORGan_P_,
                                  const int                           morphSIZ,
                                  const bool                          should_FLIP_Y,
                                  const bool                          kickOut_SkinColored );

    bool   PARAM_applySKIN_2_PCL_backProjection;
    double PARAM_SkinnDetector_THRESH;
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat load_YML_img( std::string PATH );
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    QString syntheticORrealistic;
    int     sequenceID;
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    bool reprojectPoint( const TYPE_Point_PostPr &pt, int &u, int &v );
    bool reprojectPoint( const TYPE_Point_Sensor &pt, int &u, int &v );

};

#endif // REGISTRATOR_H
