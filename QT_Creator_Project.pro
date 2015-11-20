QT += core

TARGET   = reconstructionFromHandObjectInteractions
TEMPLATE = app


CONFIG(debug, debug|release){
    DESTDIR     = ./bin/debug
    OBJECTS_DIR =   build/debug/obj     # .obj
    MOC_DIR     =   build/debug/moc     # .moc
    RCC_DIR     =   build/debug/rcc     # .rcc
    UI_DIR      =   build/debug/ui      # .ui
}

CONFIG(release, debug|release){
    DESTDIR     = ./bin/release
    OBJECTS_DIR =   build/release/obj   # .obj
    MOC_DIR     =   build/release/moc   # .moc
    RCC_DIR     =   build/release/rcc   # .rcc
    UI_DIR      =   build/release/ui    # .ui
}


INCLUDEPATH += .                \
               $$PWD/include    \
               $$PWD/src

DEPENDPATH  += .                \
               $$PWD/include    \
               $$PWD/src


SOURCES +=  src/main.cpp                                    \
            src/model.cpp                                   \
            src/model_Skeleton.cpp                          \
            src/model_Skin.cpp                              \
            src/modelSET.cpp                                \
            src/animation_Transform.cpp                     \
            src/animation_Files_CompleteSequence.cpp        \
            src/animation.cpp                               \
            src/registrator.cpp                             \
            src/registrator_Help_1_NormalEstimator.cpp      \
            src/registrator_Help_3_FeatureDescription.cpp   \
            src/registrator_Help_6_Filtering_Voxelgrid.cpp  \
            src/registrator_RegNEW_x_CloudResolution.cpp    \
            src/registrator_Help_5_ICP_Refinement.cpp       \
            src/registrator_Help_4_CorrEstimRej.cpp         \
            src/registrator_Help_2_KeyPointDetection.cpp    \
            src/registrator_Help_0_BilateralFiltering.cpp   \
            src/sequence.cpp                                \
            src/cameraSet.cpp                               \
            src/touch.cpp                                   \
            src/touch_Detector_Helpers.cpp                  \
            src/model_Mesh.cpp                              \
            src/model_Extra_SkinningStuff.cpp               \
            src/renderer.cpp                                \
            src/model_Extra_KinematicChain.cpp              \
            src/touch_Correspondences_SKIN.cpp              \
            src/registrator_RegNEW_0_Conv__IMG_2_PCL.cpp    \
            src/registrator_0_CONFIGs.cpp                   \
            src/registrator_Help_7_Helpers.cpp              \
            src/animation_Touch.cpp                         \
            src/skinn_Detector_Mixture.cpp                  \
            src/skinn_Detector_DATA.cpp                     \
            src/skinn_Detector.cpp                          \
            src/touch_Detector_PCLvisible_SKIN.cpp          \
            src/touch_Distance.cpp                          \
            src/registrator_RegNEW_5_TSDF_local.cpp         \
            src/registrator_RegNEW_4_FUSE_CORRs.cpp         \
            src/registrator_RegNEW_2_LocAlign.cpp           \
            src/registrator_RegNEW_1_TouchIDs_YYY.cpp       \
            src/registrator_RegNEW_6_MESHLAB.cpp

HEADERS  += \
            include/model.h                                 \
            include/modelSET.h                              \
            include/animation.h                             \
            include/registrator.h                           \
            include/sequence.h                              \
            include/cameraSet.h                             \
            include/touch.h                                 \
            include/renderer.h                              \
            include/skinndetector.h                         \
            include/robustMatcher.h


INCLUDEPATH += /usr/include/pcl-1.7     # needed!!!
INCLUDEPATH += /usr/include/eigen3      # needed!!!
INCLUDEPATH += /usr/include/vtk-5.8     # needed!!!


LIBS += -lboost_system                  # needed!!!
LIBS += -lboost_thread                  # needed!!!

LIBS += -L/usr/lib/                     # needed!!!
LIBS += -lpcl_common                    # needed!!!
LIBS += -lpcl_features                  # needed!!!
LIBS += -lpcl_filters                   # needed!!!
LIBS += -lpcl_io                        # needed!!!
LIBS += -lpcl_registration              # needed!!!
LIBS += -lpcl_sample_consensus          # needed!!!
LIBS += -lpcl_search                    # needed!!!
LIBS += -lpcl_visualization             # needed!!!

LIBS += -lvtkCommon                     # needed!!!
LIBS += -lvtkRendering                  # needed!!!
LIBS += -lvtkGraphics                   # needed!!!
LIBS += -lvtkFiltering                  # needed!!!

LIBS += -lconfig++

LIBS += -L/usr/local/lib/               # needed!!!
LIBS += -lopencv_core                   # needed!!!
LIBS += -lopencv_imgproc                # needed!!!
LIBS += -lopencv_highgui                # needed!!!
LIBS += -lopencv_features2d             # needed!!!
LIBS += -lopencv_nonfree                # needed!!!
LIBS += -lopencv_calib3d                # needed!!!


INCLUDEPATH += $$PWD/3rd_party/cpu_tsdf/include
LIBS        += -lcpu_tsdf

CONFIG += console

CONFIG += debug_and_release
CONFIG += warn_off

OTHER_FILES += src/registrator_RegNEW_6_MESLAB_SCRIPT.mlx
