// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef TOUCH_H
#define TOUCH_H

#include <QVector>

#include <sequence.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/distances.h>

#include <pcl/common/common.h>
#include <pcl/correspondence.h>


struct TouchCorresp_SKELETON
{
    TouchingJoint source;
    TouchingJoint target;
};
struct TouchCorresp_SKIN
{
    TouchingVertex source;
    TouchingVertex target;
};


struct SphereCoff
{
    float x;
    float y;
    float z;
    float radius;
    pcl::PointXYZ   center_PointXYZ;
    Eigen::VectorXf eigen_4x1;
};


class Touch
{
public:
    Touch();
    Touch( Sequence* sequence  );
           Sequence* sequence;

    ////////////////////////////////////////////////////////
    // typedef pcl::PointXYZRGB        TYPE_Point_Sensor; //
    // typedef pcl::PointXYZRGBNormal  TYPE_Point_PostPr; //
    ////////////////////////////////////////////////////////
    int  detector_SkinTouch( int animNumb, pcl::search::KdTree<pcl::PointXYZRGBNormal> &tree, bool should_ONLY_EF, bool shouldPrint );

    void detector_SkinTouch_ExpandTouchingBones_at_FULL_BONE();

    void correspondencesFinder_SKIN_full();
    void correspondencesFinder_SKIN_union();
    void correspondencesFinder_SKIN_intersection();
    bool correspondencesFinder_SKIN_ENSURE_2_FINGs();
    void correspondencesPrinter_SKIN();

    void TouchCorrespSKIN_TRASNFORM_Source( Eigen::Matrix4f transformationMatrix4f );
    void TouchCorrespSKIN_TRASNFORM_Target( Eigen::Matrix4f transformationMatrix4f );

    QVector<TouchCorresp_SKIN>  myTouchCorrespSKIN;

    double getDistance_JOINT_to_PCL_from_sensor( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_PCLsensor,  pcl::PointXYZ &bone_Joint_PclPointXYZ );
    double getDistance_JOINT_to_SpherePrimitive( SphereCoff                          &sphereCoff,          pcl::PointXYZ &bone_Joint_PclPointXYZ );

    double getDistance_SKIN_to_PCL_from_sensor(  pcl::search::KdTree<pcl::PointXYZRGBNormal>  &tree,       pcl::PointXYZRGBNormal &cloudPt_SKIN );
    double getDistance_SKIN_to_SpherePrimitive(  SphereCoff                                   &sphereCoff, pcl::PointXYZRGBNormal &cloudPt_SKIN );

    void printer_AllBones_OneAnim( int animNumb );
    void printer_Minimal__AllAnim();

    double  PARAM_SKIN_Dist_THRESH;
    double  PARAM_SKIN_Dist_THRESH_CURR;
    double  PARAM_SKIN_Dist_THRESH_INCR;
    int     PARAM_SKIN_minTouchingFINGs;
    int     PARAM_SKIN_minTouchingPointsPerFinger;

    bool    PARAM_SKIN_ENFORCE_MIN_TouchFING_2_EXIT;
    bool    PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger;
    QString PARAM_TOUCH_Mode__intersection_OR_union_OR_full;

    bool    PARAM_DBG_RenderMesh_Color_TouchCORR;

    bool    PARAM_should_ONLY_EF;

    int     skinTouch_countDiffFingers_COMMON();
    int     skinTouch_countDiffFingers_PER_ANIM(     int             animID );
    void    skinTouch_fingID_2_globalFingID(         const QString&  boneName, int& modelID, int& fingID, int& globalFingID );
    bool    skinTouch_isFingerInTouch(               const int&                                                globalFingID );
    QString skinTouch_printTouchingFingers();


    void TouchingVertices_TRASNFORM_Source(  Eigen::Matrix4f transformationMatrix4f );
    void TouchingVertices_TRASNFORM_Target(  Eigen::Matrix4f transformationMatrix4f );

    void TouchingVertices_PRINT_Source();
    void TouchingVertices_PRINT_Target();

    int  TouchingVertices_READ_Source(       QString FULL_PATH_READ );
    int  TouchingVertices_READ_Target(       QString FULL_PATH_READ );

    void TouchingVertices_WRITE_Source(      QString FULL_PATH_SAVE );
    void TouchingVertices_WRITE_Target(      QString FULL_PATH_SAVE );

    void TouchingVertices_PRINT_DBG(                            const QVector< TouchingVertex > &myTouchingVertices );
    int  TouchingVertices_READ_DBG(   QString FULL_PATH_READ,         QVector< TouchingVertex > &myTouchingVertices );

private:

    void correspondencesFinder_SKIN_union( QVector<TouchingVertex>& touchingVertices_Source, ModelSet& modelSet_Source,
                                           QVector<TouchingVertex>& touchingVertices_Target, ModelSet& modelSet_Target );

    struct TouchingFinger_COUNT_element
    {
        int     modelID;
        int     fingID;
        int     globalFingID;
        int     counter;
        QString name_skinningBone;
        int          skinningBoneID;
        double  signedDist;
    };

    QVector<TouchingFinger_COUNT_element> touchingFinger_COUNTer;

};

#endif // TOUCH_H
