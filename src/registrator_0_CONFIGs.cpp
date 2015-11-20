// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "registrator.h"




void Registrator::read_CONFIG_RUN()
{

        /////////////////////////////////
        libconfig::Config configFile_RUN;
        /////////////////////////////////

        std::cout << PATH_CONFIG_RUN.toStdString() << std::endl;

        try
        {
            configFile_RUN.readFile( PATH_CONFIG_RUN.toLatin1() );
        }
        catch(const libconfig::FileIOException &fioex)   {   std::cout << "Could not read config file "                                                    << std::endl;   return;   }
        catch(      libconfig::ParseException  &pex  )   {   std::cout << "Parse error at config file, Line: " << pex.getLine() << " - " << pex.getError() << std::endl;   return;   }


        PARAM_RUN_applyHANDs           =                                  configFile_RUN.lookup( "PARAM_RUN_applyHANDs" );
        PARAM_RUN_hasHANDs             =                                  configFile_RUN.lookup( "PARAM_RUN_hasHANDs" );        /////////////////////////////////////////////////////////////////////////
        PARAM_RUN_SEQ_ID               = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_RUN_SEQ_ID" ));         PARAM_RUN_INPUT_PATH = VIDEO_FRAMES___BasePath + PARAM_RUN_SEQ_ID + "/1";
        PARAM_RUN_RunningMODE          = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_RUN_RunningMODE" ));    /////////////////////////////////////////////////////////////////////////
        PARAM_RUN_VIEW_ENABLED         =                                  configFile_RUN.lookup( "PARAM_RUN_VIEW_ENABLED" );
        PARAM_RUN_FEAT_ENABLED         =                                  configFile_RUN.lookup( "PARAM_RUN_FEAT_ENABLED" );
        PARAM_RUN_ICP_ENABLED          =                                  configFile_RUN.lookup( "PARAM_RUN_ICP_ENABLED" );
        PARAM_RUN_syntheticORrealistic = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_RUN_syntheticORrealistic" ));
        PARAM_RUN_commentStr           = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_RUN_commentStr" ));

        PARAM_INDEX_List.indexPairs.resize(1);
        PARAM_INDEX_List.indexPairs[0].STARTTT_fileID = 0;
        PARAM_INDEX_List.indexPairs[0].ENDDDDD_fileID = 0;


        PARAM_KILL_BILATERAL            = configFile_RUN.lookup( "PARAM_KILL_BILATERAL"            );
        PARAM_KILL_SOR                  = configFile_RUN.lookup( "PARAM_KILL_SOR"                  );
        PARAM_KILL_PASS                 = configFile_RUN.lookup( "PARAM_KILL_PASS"                 );

        PARAM_RUN_CorrWeight_OBJ_Feat2d = configFile_RUN.lookup( "PARAM_RUN_CorrWeight_OBJ_Feat2d" );
        PARAM_RUN_CorrWeight_OBJ_Feat3d = configFile_RUN.lookup( "PARAM_RUN_CorrWeight_OBJ_Feat3d" );
        PARAM_RUN_CorrWeight_SKIN       = configFile_RUN.lookup( "PARAM_RUN_CorrWeight_SKIN"       );


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_RUN__ONLY__TOUCH__TRANSF = configFile_RUN.lookup( "PARAM_RUN__ONLY__TOUCH__TRANSF"  );
        if (PARAM_RUN_applyHANDs==false)                          PARAM_RUN__ONLY__TOUCH__TRANSF = false;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        PARAM_ICP_Mode__p2p_TRICK = configFile_RUN.lookup( "PARAM_ICP_Mode__p2p_TRICK" );

}

void Registrator::print_CONFIG_RUN()
{

        std::cout <<                                                                                                                   std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout << "PARAM_RUN_applyHANDs                          " << "\t\t" << PARAM_RUN_applyHANDs                             << std::endl;
        std::cout << "PARAM_RUN_hasHANDs                            " << "\t\t" << PARAM_RUN_hasHANDs                               << std::endl;
        std::cout << "PARAM_RUN_SEQ_ID                              " << "\t\t" << PARAM_RUN_SEQ_ID.toStdString()                   << std::endl;
        std::cout << "PARAM_RUN_INPUT_PATH                          " << "\t\t" << PARAM_RUN_INPUT_PATH.toStdString()               << std::endl;
        std::cout << "PARAM_RUN_RunningMODE                         " << "\t\t" << PARAM_RUN_RunningMODE.toStdString()              << std::endl;
        std::cout << "PARAM_RUN_VIEW_ENABLED                        " << "\t\t" << PARAM_RUN_VIEW_ENABLED                           << std::endl;
        std::cout << "PARAM_RUN_FEAT_ENABLED                        " << "\t\t" << PARAM_RUN_FEAT_ENABLED                           << std::endl;
        std::cout << "PARAM_RUN_ICP_ENABLED                         " << "\t\t" << PARAM_RUN_ICP_ENABLED                            << std::endl;
        std::cout << "PARAM_RUN_syntheticORrealistic                " << "\t\t" << PARAM_RUN_syntheticORrealistic.toStdString()     << std::endl;
        std::cout << "PARAM_RUN_commentStr                          " << "\t\t" << PARAM_RUN_commentStr.toStdString()               << std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout << "PARAM_KILL_BILATERAL                          " << "\t\t" << PARAM_KILL_BILATERAL                             << std::endl;
        std::cout << "PARAM_KILL_SOR                                " << "\t\t" << PARAM_KILL_SOR                                   << std::endl;
        std::cout << "PARAM_KILL_PASS                               " << "\t\t" << PARAM_KILL_PASS                                  << std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout << "PARAM_RUN_CorrWeight_OBJ_Feat2d               " << "\t\t" << PARAM_RUN_CorrWeight_OBJ_Feat2d                  << std::endl;
        std::cout << "PARAM_RUN_CorrWeight_OBJ_Feat3d               " << "\t\t" << PARAM_RUN_CorrWeight_OBJ_Feat3d                  << std::endl;
        std::cout << "PARAM_RUN_CorrWeight_SKIN                     " << "\t\t" << PARAM_RUN_CorrWeight_SKIN                        << std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout << "PARAM_RUN__ONLY__TOUCH__TRANSF                " << "\t\t" << PARAM_RUN__ONLY__TOUCH__TRANSF                   << std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout << "PARAM_ICP_Mode__p2p_TRICK                     " << "\t\t" << PARAM_ICP_Mode__p2p_TRICK                        << std::endl;
        std::cout <<                                                                                                                   std::endl;
        std::cout <<                                                                                                                   std::endl;

}




void Registrator::read_CONFIG_DBG()
{

        /////////////////////////////////
        libconfig::Config configFile_DBG;
        /////////////////////////////////

        std::cout << PATH_CONFIG_DBG.toStdString() << std::endl;

        try
        {
            configFile_DBG.readFile( PATH_CONFIG_DBG.toLatin1() );
        }
        catch(const libconfig::FileIOException &fioex)   {   std::cout << "Could not read config file "                                                    << std::endl;   return;   }
        catch(      libconfig::ParseException  &pex  )   {   std::cout << "Parse error at config file, Line: " << pex.getLine() << " - " << pex.getError() << std::endl;   return;   }


        PARAM_DBG_Apply_Transf_Obj_Feat2d            = configFile_DBG.lookup( "PARAM_DBG_Apply_Transf_Obj_Feat2d" );
        PARAM_DBG_Apply_Transf_Obj_Feat3d            = configFile_DBG.lookup( "PARAM_DBG_Apply_Transf_Obj_Feat3d" );
        PARAM_DBG_Apply_Transf_Skin                  = configFile_DBG.lookup( "PARAM_DBG_Apply_Transf_Skin" );
        PARAM_DBG_Apply_Transf_ICP                   = configFile_DBG.lookup( "PARAM_DBG_Apply_Transf_ICP" );

        PARAM_DBG_RenderObj_Source                   = configFile_DBG.lookup( "PARAM_DBG_RenderObj_Source" );
        PARAM_DBG_RenderObj_Target                   = configFile_DBG.lookup( "PARAM_DBG_RenderObj_Target" );

        PARAM_DBG_RenderObjNORMALS_Source            = configFile_DBG.lookup( "PARAM_DBG_RenderObjNORMALS_Source" );
        PARAM_DBG_RenderObjNORMALS_Target            = configFile_DBG.lookup( "PARAM_DBG_RenderObjNORMALS_Target" );

        PARAM_DBG_RenderSkeleton_Source              = configFile_DBG.lookup( "PARAM_DBG_RenderSkeleton_Source" );
        PARAM_DBG_RenderSkeleton_Target              = configFile_DBG.lookup( "PARAM_DBG_RenderSkeleton_Target" );

        PARAM_DBG_RenderPrimitive_Source             = configFile_DBG.lookup( "PARAM_DBG_RenderPrimitive_Source" );
        PARAM_DBG_RenderPrimitive_Target             = configFile_DBG.lookup( "PARAM_DBG_RenderPrimitive_Target" );

        PARAM_DBG_Render_PASS_FILTER                 = configFile_DBG.lookup( "PARAM_DBG_Render_PASS_FILTER" );

        PARAM_DBG_RenderCORR_Obj_Feat2d              = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Obj_Feat2d" );
        PARAM_DBG_RenderCORR_Obj_Feat3d              = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Obj_Feat3d" );
        PARAM_DBG_RenderCORR_Obj_FakeTransf          = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Obj_FakeTransf" );
        PARAM_DBG_RenderCORR_Skin                    = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Skin" );
        PARAM_DBG_RenderCORR_SkinADD                 = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_SkinADD" );

        PARAM_DBG_RenderCORR_Skin_TouchVertSphere    = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Skin_TouchVertSphere" );
        PARAM_DBG_RenderCORR_Skin_TouchVertSphereSiz = configFile_DBG.lookup( "PARAM_DBG_RenderCORR_Skin_TouchVertSphereSiz" );

        PARAM_DBG_addCoordinateSystem                = configFile_DBG.lookup( "PARAM_DBG_addCoordinateSystem" );
        PARAM_DBG_addCoordinateSysSIZ                = configFile_DBG.lookup( "PARAM_DBG_addCoordinateSysSIZ" );

        PARAM_DBG_offfff                             = configFile_DBG.lookup( "PARAM_DBG_offfff" );
        PARAM_DBG_offInter                           = configFile_DBG.lookup( "PARAM_DBG_offInter" );

        PARAM_DBG_cameraFlipX                        = configFile_DBG.lookup( "PARAM_DBG_cameraFlipX" );
        PARAM_DBG_cameraFlipY                        = configFile_DBG.lookup( "PARAM_DBG_cameraFlipY" );
        PARAM_DBG_cameraFlipZ                        = configFile_DBG.lookup( "PARAM_DBG_cameraFlipZ" );

        PARAM_DBG_RenderMesh_Source__Hands           = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Source__Hands" );
        PARAM_DBG_RenderMesh_Source__Object          = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Source__Object" );
        PARAM_DBG_RenderMesh_Source__Normals         = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Source__Normals" );
        PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1   = configFile_DBG.lookup( "PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1" );

        PARAM_DBG_RenderMesh_Target__Hands           = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Target__Hands" );
        PARAM_DBG_RenderMesh_Target__Object          = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Target__Object" );
        PARAM_DBG_RenderMesh_Target__Normals         = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Target__Normals" );
        PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1   = configFile_DBG.lookup( "PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1" );

        PARAM_DBG_RenderMesh_Source = PARAM_DBG_RenderMesh_Source__Hands || PARAM_DBG_RenderMesh_Source__Object;
        PARAM_DBG_RenderMesh_Target = PARAM_DBG_RenderMesh_Target__Hands || PARAM_DBG_RenderMesh_Target__Object;

        PARAM_DBG_Renderable_Model_Hand_R            = configFile_DBG.lookup( "PARAM_DBG_Renderable_Model_Hand_R" );
        PARAM_DBG_Renderable_Model_Hand_L            = configFile_DBG.lookup( "PARAM_DBG_Renderable_Model_Hand_L" );
        PARAM_DBG_Renderable_Model_Object            = configFile_DBG.lookup( "PARAM_DBG_Renderable_Model_Object" );

        PARAM_DBG_OverloadPCL_Source_ALL             = configFile_DBG.lookup( "PARAM_DBG_OverloadPCL_Source_ALL" );
        PARAM_DBG_OverloadPCL_Target_ALL             = configFile_DBG.lookup( "PARAM_DBG_OverloadPCL_Target_ALL" );
        PARAM_DBG_OverloadPCL_Source_HandsNonOccl    = configFile_DBG.lookup( "PARAM_DBG_OverloadPCL_Source_HandsNonOccl" );
        PARAM_DBG_OverloadPCL_Target_HandsNonOccl    = configFile_DBG.lookup( "PARAM_DBG_OverloadPCL_Target_HandsNonOccl" );

        PARAM_DBG_OBJ_FEAT_2D_Source                 = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_2D_Source" );
        PARAM_DBG_OBJ_FEAT_2D_Target                 = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_2D_Target" );
        PARAM_DBG_OBJ_FEAT_2D_SHOW                   = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_2D_SHOW" );
        PARAM_DBG_OBJ_FEAT_2D_FLIP                   = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_2D_FLIP" );
        PARAM_DBG_OBJ_FEAT_3D_Source                 = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_3D_Source" );
        PARAM_DBG_OBJ_FEAT_3D_Target                 = configFile_DBG.lookup( "PARAM_DBG_OBJ_FEAT_3D_Target" );


                                                                PARAM_DBG_Normals_VizDistCOEFF = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")   PARAM_DBG_Normals_VizDistCOEFF = configFile_DBG.lookup( "PARAM_DBG_Normals_VizDistCOEFF_CARMINE" );
        else if (syntheticORrealistic == "synthetic")           PARAM_DBG_Normals_VizDistCOEFF = configFile_DBG.lookup( "PARAM_DBG_Normals_VizDistCOEFF_SYNTH"   );

                                                                PARAM_DBG_Normals_VizSizeCOEFF = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")   PARAM_DBG_Normals_VizSizeCOEFF = configFile_DBG.lookup( "PARAM_DBG_Normals_VizSizeCOEFF_CARMINE" );
        else if (syntheticORrealistic == "synthetic")           PARAM_DBG_Normals_VizSizeCOEFF = configFile_DBG.lookup( "PARAM_DBG_Normals_VizSizeCOEFF_SYNTH"   );

                                                                PARAM_DBG_offx = 0;
        if      (syntheticORrealistic == "realistic_CARMINE")   PARAM_DBG_offx = configFile_DBG.lookup( "PARAM_DBG_offx_realistic_CARMINE" );
        else if (syntheticORrealistic == "synthetic")           PARAM_DBG_offx = configFile_DBG.lookup( "PARAM_DBG_offx_synthetic"         );

                                                                PARAM_DBG_offy = 0;
        if      (syntheticORrealistic == "realistic_CARMINE")   PARAM_DBG_offy = configFile_DBG.lookup( "PARAM_DBG_offy_realistic_CARMINE" );
        else if (syntheticORrealistic == "synthetic")           PARAM_DBG_offy = configFile_DBG.lookup( "PARAM_DBG_offy_synthetic"         );

                                                                PARAM_DBG_offz = 0;
        if      (syntheticORrealistic == "realistic_CARMINE")   PARAM_DBG_offz = configFile_DBG.lookup( "PARAM_DBG_offz_realistic_CARMINE" );
        else if (syntheticORrealistic == "synthetic")           PARAM_DBG_offz = configFile_DBG.lookup( "PARAM_DBG_offz_synthetic"         );

        if (applyHANDs)
        {
                touch->PARAM_DBG_RenderMesh_Color_TouchCORR = configFile_DBG.lookup( "PARAM_DBG_RenderMesh_Color_TouchCORR" );
        }

}

void Registrator::print_CONFIG_DBG()
{

        std::cout <<                                                                                                           std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_Apply_Transf_Obj_Feat2d         " << "\t\t" << PARAM_DBG_Apply_Transf_Obj_Feat2d            << std::endl;
        std::cout << "PARAM_DBG_Apply_Transf_Obj_Feat3d         " << "\t\t" << PARAM_DBG_Apply_Transf_Obj_Feat3d            << std::endl;
        std::cout << "PARAM_DBG_Apply_Transf_Skin               " << "\t\t" << PARAM_DBG_Apply_Transf_Skin                  << std::endl;
        std::cout << "PARAM_DBG_Apply_Transf_ICP                " << "\t\t" << PARAM_DBG_Apply_Transf_ICP                   << std::endl;
        std::cout <<                                                                                                          std::endl;
        std::cout << "PARAM_DBG_RenderObj_Source                " << "\t\t" << PARAM_DBG_RenderObj_Source                  << std::endl;
        std::cout << "PARAM_DBG_RenderObj_Target                " << "\t\t" << PARAM_DBG_RenderObj_Target                   << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderObjNORMALS_Source         " << "\t\t" << PARAM_DBG_RenderObjNORMALS_Source            << std::endl;
        std::cout << "PARAM_DBG_RenderObjNORMALS_Target         " << "\t\t" << PARAM_DBG_RenderObjNORMALS_Target            << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderSkeleton_Source           " << "\t\t" << PARAM_DBG_RenderSkeleton_Source              << std::endl;
        std::cout << "PARAM_DBG_RenderSkeleton_Target           " << "\t\t" << PARAM_DBG_RenderSkeleton_Target              << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Source               " << "\t\t" << PARAM_DBG_RenderMesh_Source                  << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Source__Hands        " << "\t\t" << PARAM_DBG_RenderMesh_Source__Hands           << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Source__Object       " << "\t\t" << PARAM_DBG_RenderMesh_Source__Object          << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Source__Normals      " << "\t\t" << PARAM_DBG_RenderMesh_Source__Normals         << std::endl;
        std::cout << "PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1" << "\t\t" << PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1   << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Target               " << "\t\t" << PARAM_DBG_RenderMesh_Target                  << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Target__Hands        " << "\t\t" << PARAM_DBG_RenderMesh_Target__Hands           << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Target__Object       " << "\t\t" << PARAM_DBG_RenderMesh_Target__Object          << std::endl;
        std::cout << "PARAM_DBG_RenderMesh_Target__Normals      " << "\t\t" << PARAM_DBG_RenderMesh_Target__Normals         << std::endl;
        std::cout << "PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1" << "\t\t" << PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1   << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_Renderable_Model_Hand_R         " << "\t\t" << PARAM_DBG_Renderable_Model_Hand_R            << std::endl;
        std::cout << "PARAM_DBG_Renderable_Model_Hand_L         " << "\t\t" << PARAM_DBG_Renderable_Model_Hand_L            << std::endl;
        std::cout << "PARAM_DBG_Renderable_Model_Object         " << "\t\t" << PARAM_DBG_Renderable_Model_Object            << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderPrimitive_Source          " << "\t\t" << PARAM_DBG_RenderPrimitive_Source             << std::endl;
        std::cout << "PARAM_DBG_RenderPrimitive_Target          " << "\t\t" << PARAM_DBG_RenderPrimitive_Target             << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_Render_PASS_FILTER              " << "\t\t" << PARAM_DBG_Render_PASS_FILTER                 << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Obj_Feat2d           " << "\t\t" << PARAM_DBG_RenderCORR_Obj_Feat2d              << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Obj_Feat3d           " << "\t\t" << PARAM_DBG_RenderCORR_Obj_Feat3d              << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Obj_FakeTransf       " << "\t\t" << PARAM_DBG_RenderCORR_Obj_FakeTransf          << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Skin                 " << "\t\t" << PARAM_DBG_RenderCORR_Skin                    << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_SkinADD              " << "\t\t" << PARAM_DBG_RenderCORR_SkinADD                 << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Skin_TouchVertSphere " << "\t\t" << PARAM_DBG_RenderCORR_Skin_TouchVertSphere    << std::endl;
        std::cout << "PARAM_DBG_RenderCORR_Skin_TouchVertSphereS" << "\t\t" << PARAM_DBG_RenderCORR_Skin_TouchVertSphereSiz << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_addCoordinateSystem             " << "\t\t" << PARAM_DBG_addCoordinateSystem                << std::endl;
        std::cout << "PARAM_DBG_addCoordinateSysSIZ             " << "\t\t" << PARAM_DBG_addCoordinateSysSIZ                << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_Normals_VizDistCOEFF            " << "\t\t" << PARAM_DBG_Normals_VizDistCOEFF               << std::endl;
        std::cout << "PARAM_DBG_Normals_VizSizeCOEFF            " << "\t\t" << PARAM_DBG_Normals_VizSizeCOEFF               << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_offfff                          " << "\t\t" << PARAM_DBG_offfff                             << std::endl;
        std::cout << "PARAM_DBG_offInter                        " << "\t\t" << PARAM_DBG_offInter                           << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_cameraFlipX                     " << "\t\t" << PARAM_DBG_cameraFlipX                        << std::endl;
        std::cout << "PARAM_DBG_cameraFlipY                     " << "\t\t" << PARAM_DBG_cameraFlipY                        << std::endl;
        std::cout << "PARAM_DBG_cameraFlipZ                     " << "\t\t" << PARAM_DBG_cameraFlipZ                        << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_offx                            " << "\t\t" << PARAM_DBG_offx                               << std::endl;
        std::cout << "PARAM_DBG_offy                            " << "\t\t" << PARAM_DBG_offy                               << std::endl;
        std::cout << "PARAM_DBG_offz                            " << "\t\t" << PARAM_DBG_offz                               << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_OverloadPCL_Source_ALL          " << "\t\t" << PARAM_DBG_OverloadPCL_Source_ALL             << std::endl;
        std::cout << "PARAM_DBG_OverloadPCL_Target_ALL          " << "\t\t" << PARAM_DBG_OverloadPCL_Target_ALL             << std::endl;
        std::cout << "PARAM_DBG_OverloadPCL_Source_HandsNonOccl " << "\t\t" << PARAM_DBG_OverloadPCL_Source_HandsNonOccl    << std::endl;
        std::cout << "PARAM_DBG_OverloadPCL_Target_HandsNonOccl " << "\t\t" << PARAM_DBG_OverloadPCL_Target_HandsNonOccl    << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_2D_Source              " << "\t\t" << PARAM_DBG_OBJ_FEAT_2D_Source                 << std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_2D_Target              " << "\t\t" << PARAM_DBG_OBJ_FEAT_2D_Target                 << std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_2D_SHOW                " << "\t\t" << PARAM_DBG_OBJ_FEAT_2D_SHOW                   << std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_2D_FLIP                " << "\t\t" << PARAM_DBG_OBJ_FEAT_2D_FLIP                   << std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_3D_Source              " << "\t\t" << PARAM_DBG_OBJ_FEAT_3D_Source                 << std::endl;
        std::cout << "PARAM_DBG_OBJ_FEAT_3D_Target              " << "\t\t" << PARAM_DBG_OBJ_FEAT_3D_Target                 << std::endl;
        std::cout <<                                                                                                           std::endl;


        if (applyHANDs)
        {
        std::cout << "PARAM_DBG_RenderMesh_Color_TouchCORR      " << "\t\t" << touch->PARAM_DBG_RenderMesh_Color_TouchCORR  << std::endl;
        }

        std::cout <<                                                                                                           std::endl;
        std::cout <<                                                                                                           std::endl;

}



void Registrator::PARAM_DBG_flags__regulateActivation( bool TEST_MODE )
{

        PARAM_DBG_Apply_Transf_Obj_Feat2d           &= TEST_MODE;
        PARAM_DBG_Apply_Transf_Obj_Feat3d           &= TEST_MODE;
        PARAM_DBG_Apply_Transf_Skin                 &= TEST_MODE;
        PARAM_DBG_Apply_Transf_ICP                  &= TEST_MODE;

        PARAM_DBG_RenderObj_Source                  &= TEST_MODE;
        PARAM_DBG_RenderObj_Target                  &= TEST_MODE;

        PARAM_DBG_RenderObjNORMALS_Source           &= TEST_MODE;
        PARAM_DBG_RenderObjNORMALS_Target           &= TEST_MODE;

        PARAM_DBG_RenderSkeleton_Source             &= TEST_MODE;
        PARAM_DBG_RenderSkeleton_Target             &= TEST_MODE;

        PARAM_DBG_RenderMesh_Source                 &= TEST_MODE;
        PARAM_DBG_RenderMesh_Source__Hands          &= TEST_MODE;
        PARAM_DBG_RenderMesh_Source__Object         &= TEST_MODE;
        PARAM_DBG_RenderMesh_Source__Normals        &= TEST_MODE;
        PARAM_DBG_ColorrMesh_Source__prAnim0_Mdel1  &= TEST_MODE;

        PARAM_DBG_RenderMesh_Target                 &= TEST_MODE;
        PARAM_DBG_RenderMesh_Target__Hands          &= TEST_MODE;
        PARAM_DBG_RenderMesh_Target__Object         &= TEST_MODE;
        PARAM_DBG_RenderMesh_Target__Normals        &= TEST_MODE;
        PARAM_DBG_ColorrMesh_Target__prAnim0_Mdel1  &= TEST_MODE;

        PARAM_DBG_Renderable_Model_Hand_R           &= TEST_MODE;
        PARAM_DBG_Renderable_Model_Hand_L           &= TEST_MODE;
        PARAM_DBG_Renderable_Model_Object           &= TEST_MODE;

        PARAM_DBG_RenderPrimitive_Source            &= TEST_MODE;
        PARAM_DBG_RenderPrimitive_Target            &= TEST_MODE;

        PARAM_DBG_RenderCORR_Obj_Feat2d             &= TEST_MODE;
        PARAM_DBG_RenderCORR_Obj_Feat3d             &= TEST_MODE;
        PARAM_DBG_RenderCORR_Obj_FakeTransf         &= TEST_MODE;
        PARAM_DBG_RenderCORR_Skin                   &= TEST_MODE;
        PARAM_DBG_RenderCORR_SkinADD                &= TEST_MODE;
        PARAM_DBG_RenderCORR_Skin_TouchVertSphere   &= TEST_MODE;

        PARAM_DBG_addCoordinateSystem               &= TEST_MODE;

        PARAM_DBG_cameraFlipX                       &= TEST_MODE;
        PARAM_DBG_cameraFlipY                       &= TEST_MODE;
        PARAM_DBG_cameraFlipZ                       &= TEST_MODE;

        PARAM_DBG_OverloadPCL_Source_ALL            &= TEST_MODE;
        PARAM_DBG_OverloadPCL_Target_ALL            &= TEST_MODE;
        PARAM_DBG_OverloadPCL_Source_HandsNonOccl   &= TEST_MODE;
        PARAM_DBG_OverloadPCL_Target_HandsNonOccl   &= TEST_MODE;

        PARAM_DBG_OBJ_FEAT_2D_Source                &= TEST_MODE;
        PARAM_DBG_OBJ_FEAT_2D_Target                &= TEST_MODE;
        PARAM_DBG_OBJ_FEAT_2D_SHOW                  &= TEST_MODE;
        PARAM_DBG_OBJ_FEAT_2D_FLIP                  &= TEST_MODE;


        if (applyHANDs)
        touch->PARAM_DBG_RenderMesh_Color_TouchCORR &= TEST_MODE;


        if (TEST_MODE == false)
        {
              PARAM_DBG_offfff   = 0;
              PARAM_DBG_offInter = 0;

              PARAM_DBG_offx     = 0;
              PARAM_DBG_offy     = 0;
              PARAM_DBG_offz     = 0;

              PARAM_DBG_Normals_VizDistCOEFF = 0;
              PARAM_DBG_Normals_VizSizeCOEFF = 0;

              PARAM_DBG_addCoordinateSysSIZ  = 0;

              PARAM_DBG_RenderCORR_Skin_TouchVertSphereSiz = 0;
        }

}




void Registrator::read_CONFIG_PARAMs()
{

        ////////////////////////////////////
        libconfig::Config configFile_PARAMs;
        ////////////////////////////////////

        std::cout << PATH_CONFIG_PARAMs.toStdString() << std::endl;

        try
        {
            configFile_PARAMs.readFile( PATH_CONFIG_PARAMs.toLatin1() );
        }
        catch(const libconfig::FileIOException &fioex)   {   std::cout << "Could not read config file "                                                    << std::endl;   return;   }
        catch(      libconfig::ParseException  &pex  )   {   std::cout << "Parse error at config file, Line: " << pex.getLine() << " - " << pex.getError() << std::endl;   return;   }


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_MORPH_ErodeSize = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_MORPH_ErodeSize = configFile_PARAMs.lookup( "CARMINE__PARAM_MORPH_ErodeSize" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                                                                  PARAM_NORMAL_MaxDepthChangeFactor     = 0.0;
                                                                  PARAM_NORMAL_MaxDepthChangeFactor     = configFile_PARAMs.lookup( "PARAM_NORMAL_MaxDepthChangeFactor" );    // The depth change threshold for computing object borders.
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_NORMAL_NormalSmoothingSize = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_NORMAL_NormalSmoothingSize = configFile_PARAMs.lookup( "CARMINE__PARAM_NORMAL_NormalSmoothingSize" ); // *resolution
        else if (syntheticORrealistic == "synthetic")             PARAM_NORMAL_NormalSmoothingSize = configFile_PARAMs.lookup( "SYNTH____PARAM_NORMAL_NormalSmoothingSize" ); // *resolution
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_KEYP_coeffSalientRadius = 0.0;
                                                                  PARAM_KEYP_coeffNonMaxRadiuss = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")  {  PARAM_KEYP_coeffSalientRadius = configFile_PARAMs.lookup( "CARMINE__PARAM_KEYP_coeffSalientRadius" );      // *resolution
                                                                  PARAM_KEYP_coeffNonMaxRadiuss = configFile_PARAMs.lookup( "CARMINE__PARAM_KEYP_coeffNonMaxRadiuss" ); }    // *resolution
        else if (syntheticORrealistic == "synthetic")          {  PARAM_KEYP_coeffSalientRadius = configFile_PARAMs.lookup( "SYNTH____PARAM_KEYP_coeffSalientRadius" );      // *resolution
                                                                  PARAM_KEYP_coeffNonMaxRadiuss = configFile_PARAMs.lookup( "SYNTH____PARAM_KEYP_coeffNonMaxRadiuss" ); }    // *resolution
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_FEAT_coeffRadiusSearch = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_FEAT_coeffRadiusSearch = configFile_PARAMs.lookup( "CARMINE__PARAM_FEAT_coeffRadiusSearch" );        // 30
        else if (syntheticORrealistic == "synthetic")             PARAM_FEAT_coeffRadiusSearch = configFile_PARAMs.lookup( "SYNTH____PARAM_FEAT_coeffRadiusSearch" );        // *resolution  // IMPORTANT: must be larger than radius used to estimate normals!!!

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_CORR_REJ_InlierThreshold_LOCAL  = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_CORR_REJ_InlierThreshold_LOCAL  = configFile_PARAMs.lookup( "CARMINE__PARAM_CORR_REJ_InlierThreshold_LOCAL"  );
        else if (syntheticORrealistic == "synthetic")             PARAM_CORR_REJ_InlierThreshold_LOCAL  = configFile_PARAMs.lookup( "SYNTH____PARAM_CORR_REJ_InlierThreshold_LOCAL"  ); // Set the maximum distance between corresponding points.
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_CORR_REJ_Kick_1_to_1            = configFile_PARAMs.lookup( "PARAM_CORR_REJ_Kick_1_to_1"         );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_CORR_FEAT2D_PerformRansac2D     = configFile_PARAMs.lookup( "PARAM_CORR_FEAT2D_PerformRansac2D"  );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_CORR_REJ_MaximumIterations = 0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_CORR_REJ_MaximumIterations = configFile_PARAMs.lookup( "CARMINE__PARAM_CORR_REJ_MaximumIterations" );
        else if (syntheticORrealistic == "synthetic")             PARAM_CORR_REJ_MaximumIterations = configFile_PARAMs.lookup( "SYNTH____PARAM_CORR_REJ_MaximumIterations" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_SOR_MeanK           = 0;
                                                                  PARAM_SOR_StddevMulThresh = 0.0;
                                                                  PARAM_SOR_MeanK           = configFile_PARAMs.lookup( "PARAM_SOR_MeanK" );
                                                                  PARAM_SOR_StddevMulThresh = configFile_PARAMs.lookup( "PARAM_SOR_StddevMulThresh" );

                                                                  PARAM_PASS_FILTER_XXX_min = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_XXX_min" );
                                                                  PARAM_PASS_FILTER_XXX_max = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_XXX_max" );
                                                                  PARAM_PASS_FILTER_YYY_min = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_YYY_min" );
                                                                  PARAM_PASS_FILTER_YYY_max = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_YYY_max" );
                                                                  PARAM_PASS_FILTER_ZZZ_min = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_ZZZ_min" );
                                                                  PARAM_PASS_FILTER_ZZZ_max = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_ZZZ_max" );
                                                                  PARAM_PASS_FILTER_Sph_SIZ = configFile_PARAMs.lookup( "PARAM_PASS_FILTER_Sph_SIZ" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_VOXELFILTER_leafSize      = 0.0; // DON'T read this, will be computed in code ;)
                                                                  PARAM_VOXELFILTER_leafSizeCOEFF = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_VOXELFILTER_leafSizeCOEFF = configFile_PARAMs.lookup( "CARMINE__PARAM_VOXELFILTER_leafSizeCOEFF" ); // 1;
        else if (syntheticORrealistic == "synthetic")             PARAM_VOXELFILTER_leafSizeCOEFF = configFile_PARAMs.lookup( "SYNTH____PARAM_VOXELFILTER_leafSizeCOEFF" ); // 1;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_VOXELFILTER_leafSize_FINAL_MESH = 0.0;
                                                                  PARAM_VOXELFILTER_leafSize_FINAL_MESH = configFile_PARAMs.lookup( "PARAM_VOXELFILTER_leafSize_FINAL_MESH" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_BILATERAL_SigmaS = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_BILATERAL_SigmaS = configFile_PARAMs.lookup( "CARMINE__PARAM_BILATERAL_SigmaS" ); // stdev of Gaussian -  spatial neighborhood/window
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_BILATERAL_SigmaR = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_BILATERAL_SigmaR = configFile_PARAMs.lookup( "CARMINE__PARAM_BILATERAL_SigmaR" ); // stdev of Gaussian - how much an adjacent pixel is downweighted due to the intensity diff (*DEPTH* in our case).
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_ICP_RansacOutlierRejectionThreshold = 0.0;
                                                                  PARAM_ICP_RansacOutlierRej__TRANSITION    = 0.0;
                                                                  PARAM_ICP_TransformationEpsilon           = 0.0;
                                                                  PARAM_ICP_EuclideanFitnessEpsilon         = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")  {  PARAM_ICP_RansacOutlierRejectionThreshold = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_RansacOutlierRejectionThreshold" );
                                                                  PARAM_ICP_RansacOutlierRej__TRANSITION    = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_RansacOutlierRej__TRANSITION"    );
                                                                  PARAM_ICP_RansacOutlierRej__touchIDs      = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_RansacOutlierRej__touchIDs"      );
                                                                  PARAM_ICP_TransformationEpsilon           = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_TransformationEpsilon"   );
                                                                  PARAM_ICP_EuclideanFitnessEpsilon         = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_EuclideanFitnessEpsilon" );
                                                                  PARAM_ICP_RansacOutlierRej_p2p_TRICK      = configFile_PARAMs.lookup( "CARMINE__PARAM_ICP_RansacOutlierRej_p2p_TRICK" );   }
        else if (syntheticORrealistic == "synthetic")          {  PARAM_ICP_RansacOutlierRejectionThreshold = configFile_PARAMs.lookup( "SYNTH____PARAM_ICP_RansacOutlierRejectionThreshold" );
                                                                  PARAM_ICP_RansacOutlierRej__TRANSITION    = configFile_PARAMs.lookup( "SYNTH____PARAM_ICP_RansacOutlierRej__TRANSITION"    );
                                                                  PARAM_ICP_TransformationEpsilon           = configFile_PARAMs.lookup( "SYNTH____PARAM_ICP_TransformationEpsilon"   );
                                                                  PARAM_ICP_EuclideanFitnessEpsilon         = configFile_PARAMs.lookup( "SYNTH____PARAM_ICP_EuclideanFitnessEpsilon" );      }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_ICP_MAX_ITER = 0;
                                                                  PARAM_ICP_MAX_ITER = configFile_PARAMs.lookup( "PARAM_ICP_MAX_ITER" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_GridSize   = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_GridSize"   );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_GridSize   = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_GridSize"   );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_Resolution = 0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_Resolution = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_Resolution" );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_Resolution = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_Resolution" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_Color          = false;
                                                                  PARAM_TSDF_ColorSetByConf = false;
                                                                  PARAM_TSDF_Color          = configFile_PARAMs.lookup( "PARAM_TSDF_Color" );
                                                                  PARAM_TSDF_ColorSetByConf = configFile_PARAMs.lookup( "PARAM_TSDF_ColorSetByConf" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_DepthTrunc_MaxPOS   = 0.0;
                                                                  PARAM_TSDF_DepthTrunc_MaxNEG   = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_DepthTrunc_MaxPOS   = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_DepthTrunc_MaxPOS"   );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_DepthTrunc_MaxPOS   = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_DepthTrunc_MaxPOS"   );
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_DepthTrunc_MaxNEG   = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_DepthTrunc_MaxNEG"   );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_DepthTrunc_MaxNEG   = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_DepthTrunc_MaxNEG"   );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_SensorDistBound_MIN = 0.0;
                                                                  PARAM_TSDF_SensorDistBound_MAX = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_SensorDistBound_MIN = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_SensorDistBound_MIN" );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_SensorDistBound_MIN = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_SensorDistBound_MIN" );
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_SensorDistBound_MAX = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_SensorDistBound_MAX" );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_SensorDistBound_MAX = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_SensorDistBound_MAX" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_CAM_Intrinsics_fx = 0.0;
                                                                  PARAM_CAM_Intrinsics_fy = 0.0;
                                                                  PARAM_CAM_Intrinsics_px = 0.0;
                                                                  PARAM_CAM_Intrinsics_py = 0.0;
                                                                  PARAM_CAM_Intrinsics_fx = configFile_PARAMs.lookup( "PARAM_CAM_Intrinsics_fx" );
                                                                  PARAM_CAM_Intrinsics_fy = configFile_PARAMs.lookup( "PARAM_CAM_Intrinsics_fy" );
                                                                  PARAM_CAM_Intrinsics_px = configFile_PARAMs.lookup( "PARAM_CAM_Intrinsics_px" );
                                                                  PARAM_CAM_Intrinsics_py = configFile_PARAMs.lookup( "PARAM_CAM_Intrinsics_py" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_ISO_level      = 0.0;
                                                                  PARAM_TSDF_PercExtendGrid = 0.0;
                                                                  PARAM_TSDF_ISO_level      = configFile_PARAMs.lookup( "PARAM_TSDF_ISO_level"      );
                                                                  PARAM_TSDF_PercExtendGrid = configFile_PARAMs.lookup( "PARAM_TSDF_PercExtendGrid" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_Min_Weight_MC  = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_Min_Weight_MC  = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_Min_Weight_MC"  );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_Min_Weight_MC  = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_Min_Weight_MC"  );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_Img_Width      = 0;
                                                                  PARAM_TSDF_Img_Height     = 0;
                                                                  PARAM_TSDF_Img_Width      = configFile_PARAMs.lookup( "PARAM_TSDF_Img_Width"  );
                                                                  PARAM_TSDF_Img_Height     = configFile_PARAMs.lookup( "PARAM_TSDF_Img_Height" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_MaxWeightTruncLimit = 0;
                                                                  PARAM_TSDF_MaxWeightTruncLimit = configFile_PARAMs.lookup( "PARAM_TSDF_MaxWeightTruncLimit" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_NumRandomSplts = 0;
                                                                  PARAM_TSDF_NumRandomSplts = configFile_PARAMs.lookup( "PARAM_TSDF_NumRandomSplts" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_TSDF_MaxVoxelSize = 0.0;
        if      (syntheticORrealistic == "realistic_CARMINE")     PARAM_TSDF_MaxVoxelSize = configFile_PARAMs.lookup( "CARMINE__PARAM_TSDF_MaxVoxelSize" );
        else if (syntheticORrealistic == "synthetic")             PARAM_TSDF_MaxVoxelSize = configFile_PARAMs.lookup( "SYNTH____PARAM_TSDF_MaxVoxelSize" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_THRESH_CONV_m2mm         = configFile_PARAMs.lookup("PARAM_THRESH_CONV_m2mm");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_BackGround_RRR           = configFile_PARAMs.lookup("PARAM_BackGround_RRR");
                                                                  PARAM_BackGround_GGG           = configFile_PARAMs.lookup("PARAM_BackGround_GGG");
                                                                  PARAM_BackGround_BBB           = configFile_PARAMs.lookup("PARAM_BackGround_BBB");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_Viewer_PNG_Size_WWW      = configFile_PARAMs.lookup("PARAM_Viewer_PNG_Size_WWW");
                                                                  PARAM_Viewer_PNG_Size_HHH      = configFile_PARAMs.lookup("PARAM_Viewer_PNG_Size_HHH");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_SkinnDetector_THRESH     = configFile_PARAMs.lookup("PARAM_SkinnDetector_THRESH");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_KEYpt_min_scale           =configFile_PARAMs.lookup("PARAM_KEYpt_min_scale");
                                                                  PARAM_KEYpt_nr_octaves          =configFile_PARAMs.lookup("PARAM_KEYpt_nr_octaves");
                                                                  PARAM_KEYpt_nr_scales_per_octave=configFile_PARAMs.lookup("PARAM_KEYpt_nr_scales_per_octave");
                                                                  PARAM_KEYpt_min_contrast        =configFile_PARAMs.lookup("PARAM_KEYpt_min_contrast");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_SIFT_KEY_nFeatures         = configFile_PARAMs.lookup("PARAM_SIFT_KEY_nFeatures");
                                                                  PARAM_SIFT_KEY_nOctaveLayers     = configFile_PARAMs.lookup("PARAM_SIFT_KEY_nOctaveLayers");
                                                                  PARAM_SIFT_KEY_contrastThreshold = configFile_PARAMs.lookup("PARAM_SIFT_KEY_contrastThreshold");
                                                                  PARAM_SIFT_KEY_edgeThreshold     = configFile_PARAMs.lookup("PARAM_SIFT_KEY_edgeThreshold");
                                                                  PARAM_SIFT_KEY_sigma             = configFile_PARAMs.lookup("PARAM_SIFT_KEY_sigma");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_RMATCHER_Ratio                  = configFile_PARAMs.lookup("PARAM_RMATCHER_Ratio");
                                                                  PARAM_RMATCHER_MinDistanceToEpipolar  = configFile_PARAMs.lookup("PARAM_RMATCHER_MinDistanceToEpipolar");
                                                                  PARAM_RMATCHER_ConfidenceLevel        = configFile_PARAMs.lookup("PARAM_RMATCHER_ConfidenceLevel");
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                  PARAM_RUN_EXTRA_DBG_COUT              = configFile_PARAMs.lookup( "PARAM_RUN_EXTRA_DBG_COUT" );
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



        if (applyHANDs)
        {
                                                                          touch->PARAM_SKIN_Dist_THRESH          = 0.0;
                if      (syntheticORrealistic == "realistic_CARMINE")     touch->PARAM_SKIN_Dist_THRESH          = configFile_PARAMs.lookup( "CARMINE__PARAM_SKIN_Dist_THRESH" );
                else if (syntheticORrealistic == "synthetic")             touch->PARAM_SKIN_Dist_THRESH          = configFile_PARAMs.lookup( "SYNTH____PARAM_SKIN_Dist_THRESH" );
                                                                          touch->PARAM_SKIN_Dist_THRESH_CURR     = touch->PARAM_SKIN_Dist_THRESH;

                                                                          touch->PARAM_SKIN_Dist_THRESH_INCR     = 0.0;
                if      (syntheticORrealistic == "realistic_CARMINE")     touch->PARAM_SKIN_Dist_THRESH_INCR     = configFile_PARAMs.lookup( "CARMINE__PARAM_SKIN_Dist_THRESH_INCR" );
                else if (syntheticORrealistic == "synthetic")             touch->PARAM_SKIN_Dist_THRESH_INCR     = configFile_PARAMs.lookup( "SYNTH____PARAM_SKIN_Dist_THRESH_INCR" );

                touch->PARAM_SKIN_minTouchingPointsPerFinger    = configFile_PARAMs.lookup( "PARAM_SKIN_minTouchingPointsPerFinger" );
                touch->PARAM_SKIN_minTouchingFINGs              = configFile_PARAMs.lookup( "PARAM_SKIN_minTouchingFINGs"           );
                touch->PARAM_should_ONLY_EF                     = configFile_PARAMs.lookup( "PARAM_should_ONLY_EF"                  );

        }


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /////////////////////////////////////
        // PARAM_ICP_TransformationEpsilon //
        /////////////////////////////////////
        // Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order
        // for an optimization to be considered as having converged to the final solution.

        ///////////////////////////////////////
        // PARAM_ICP_EuclideanFitnessEpsilon //
        ///////////////////////////////////////
        // Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged.
        // The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.



        /////////////////////////////////
        libconfig::Config configFile_RUN;
        /////////////////////////////////

        std::cout << PATH_CONFIG_RUN.toStdString() << std::endl;

        try
        {
            configFile_RUN.readFile( PATH_CONFIG_RUN.toLatin1() );
        }
        catch(const libconfig::FileIOException &fioex)   {   std::cout << "Could not read config file "                                                    << std::endl;   return;   }
        catch(      libconfig::ParseException  &pex  )   {   std::cout << "Parse error at config file, Line: " << pex.getLine() << " - " << pex.getError() << std::endl;   return;   }

            PARAM_ICP_Mode__previous_OR_metascan  = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_ICP_Mode__previous_OR_metascan" ));
        if (PARAM_ICP_Mode__previous_OR_metascan != "previous" &&
            PARAM_ICP_Mode__previous_OR_metascan != "metascan" )
        {
            std::cout << "\n\n" << "PARAM_ICP_Mode__previous_OR_metascan - Invalid Value !!!" << "\n" << std::endl;
            exit(1);
        }

            PARAM_ICP_Mode__p2p_OR_p2plane             = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_ICP_Mode__p2p_OR_p2plane"            ));
            PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION  = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION" ));
        if (PARAM_ICP_Mode__p2p_OR_p2plane            != "p2p"     &&
            PARAM_ICP_Mode__p2p_OR_p2plane            != "p2plane" &&
            PARAM_ICP_Mode__p2p_OR_p2plane            != "p2planeW"&&
            PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION != "p2p"     &&
            PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION != "p2plane" &&
            PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION != "p2planeW")
        {
            std::cout << "\n\n" << "PARAM_ICP_Mode__p2p_OR_p2plane - Invalid Value !!!" << "\n" << std::endl;
            exit(1);
        }

            PARAM_OBJ_Mode__p2p_OR_p2plane  = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_OBJ_Mode__p2p_OR_p2plane" ));
        if (PARAM_OBJ_Mode__p2p_OR_p2plane != "p2p"     &&
            PARAM_OBJ_Mode__p2p_OR_p2plane != "p2plane" &&
            PARAM_OBJ_Mode__p2p_OR_p2plane != "p2planeW" )
        {
            std::cout << "\n\n" << "PARAM_OBJ_Mode__p2p_OR_p2plane - Invalid Value !!!" << "\n" << std::endl;
            exit(1);
        }

        if (applyHANDs)
        {
        touch->PARAM_SKIN_ENFORCE_MIN_TouchFING_2_EXIT          =                                  configFile_RUN.lookup( "PARAM_SKIN_ENFORCE_MIN_TouchFING_2_EXIT"          );
        touch->PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger         =                                  configFile_RUN.lookup( "PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger"         );
        touch->PARAM_TOUCH_Mode__intersection_OR_union_OR_full  = QString::fromLatin1((const char*)configFile_RUN.lookup( "PARAM_TOUCH_Mode__intersection_OR_union_OR_full"  ));
        if (touch->PARAM_TOUCH_Mode__intersection_OR_union_OR_full != "union"        &&
            touch->PARAM_TOUCH_Mode__intersection_OR_union_OR_full != "intersection" &&
            touch->PARAM_TOUCH_Mode__intersection_OR_union_OR_full != "full"          )
        {
            std::cout << "\n\n" << "PARAM_TOUCH_Mode__intersection_OR_union_OR_full - Invalid Value !!!" << "\n" << std::endl;
            exit(1);
        }
        }

}

void Registrator::print_CONFIG_PARAMs()
{

        std::cout << "PARAM_MORPH_ErodeSize                     " << "\t\t" << PARAM_MORPH_ErodeSize                        << std::endl;
        std::cout << "PARAM_NORMAL_MaxDepthChangeFactor         " << "\t\t" << PARAM_NORMAL_MaxDepthChangeFactor            << std::endl;
        std::cout << "PARAM_NORMAL_NormalSmoothingSize          " << "\t\t" << PARAM_NORMAL_NormalSmoothingSize             << std::endl;
        std::cout << "PARAM_DBG_Normals_VizDistCOEFF            " << "\t\t" << PARAM_DBG_Normals_VizDistCOEFF               << std::endl;
        std::cout << "PARAM_DBG_Normals_VizSizeCOEFF            " << "\t\t" << PARAM_DBG_Normals_VizSizeCOEFF               << std::endl;
        std::cout << "PARAM_KEYP_coeffSalientRadius             " << "\t\t" << PARAM_KEYP_coeffSalientRadius                << std::endl;
        std::cout << "PARAM_KEYP_coeffNonMaxRadiuss             " << "\t\t" << PARAM_KEYP_coeffNonMaxRadiuss                << std::endl;
        std::cout << "PARAM_FEAT_coeffRadiusSearch              " << "\t\t" << PARAM_FEAT_coeffRadiusSearch                 << std::endl;
        std::cout << "PARAM_CORR_REJ_InlierThreshold_LOCAL      " << "\t\t" << PARAM_CORR_REJ_InlierThreshold_LOCAL         << std::endl;
        std::cout << "PARAM_CORR_REJ_Kick_1_to_1                " << "\t\t" << PARAM_CORR_REJ_Kick_1_to_1                   << std::endl;
        std::cout << "PARAM_CORR_REJ_MaximumIterations          " << "\t\t" << PARAM_CORR_REJ_MaximumIterations             << std::endl;
        std::cout << "PARAM_VOXELFILTER_leafSizeCOEFF           " << "\t\t" << PARAM_VOXELFILTER_leafSizeCOEFF              << std::endl;
        std::cout << "PARAM_VOXELFILTER_leafSize_FINAL_MESH     " << "\t\t" << PARAM_VOXELFILTER_leafSize_FINAL_MESH        << std::endl;
        std::cout << "PARAM_BILATERAL_SigmaS                    " << "\t\t" << PARAM_BILATERAL_SigmaS                       << std::endl;
        std::cout << "PARAM_BILATERAL_SigmaR                    " << "\t\t" << PARAM_BILATERAL_SigmaR                       << std::endl;
        std::cout << "PARAM_ICP_RansacOutlierRejectionThreshold " << "\t\t" << PARAM_ICP_RansacOutlierRejectionThreshold    << std::endl;
        std::cout << "PARAM_ICP_RansacOutlierRej__TRANSITION    " << "\t\t" << PARAM_ICP_RansacOutlierRej__TRANSITION       << std::endl;
        std::cout << "PARAM_ICP_RansacOutlierRej__touchIDs      " << "\t\t" << PARAM_ICP_RansacOutlierRej__touchIDs         << std::endl;
        std::cout << "PARAM_ICP_RansacOutlierRej_p2p_TRICK      " << "\t\t" << PARAM_ICP_RansacOutlierRej_p2p_TRICK         << std::endl;
        std::cout << "PARAM_ICP_TransformationEpsilon           " << "\t\t" << PARAM_ICP_TransformationEpsilon              << std::endl;
        std::cout << "PARAM_ICP_EuclideanFitnessEpsilon         " << "\t\t" << PARAM_ICP_EuclideanFitnessEpsilon            << std::endl;
        std::cout << "PARAM_ICP_MAX_ITER                        " << "\t\t" << PARAM_ICP_MAX_ITER                           << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_SOR_MeanK                           " << "\t\t" << PARAM_SOR_MeanK                              << std::endl;
        std::cout << "PARAM_SOR_StddevMulThresh                 " << "\t\t" << PARAM_SOR_StddevMulThresh                    << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_PASS_FILTER_XXX_min                 " << "\t\t" << PARAM_PASS_FILTER_XXX_min                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_XXX_max                 " << "\t\t" << PARAM_PASS_FILTER_XXX_max                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_YYY_min                 " << "\t\t" << PARAM_PASS_FILTER_YYY_min                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_YYY_max                 " << "\t\t" << PARAM_PASS_FILTER_YYY_max                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_ZZZ_min                 " << "\t\t" << PARAM_PASS_FILTER_ZZZ_min                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_ZZZ_max                 " << "\t\t" << PARAM_PASS_FILTER_ZZZ_max                    << std::endl;
        std::cout << "PARAM_PASS_FILTER_Sph_SIZ                 " << "\t\t" << PARAM_PASS_FILTER_Sph_SIZ                    << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_TSDF_GridSize                       " << "\t\t" << PARAM_TSDF_GridSize                          << std::endl;
        std::cout << "PARAM_TSDF_Resolution                     " << "\t\t" << PARAM_TSDF_Resolution                        << std::endl;
        std::cout << "PARAM_TSDF_Color                          " << "\t\t" << PARAM_TSDF_Color                             << std::endl;
        std::cout << "PARAM_TSDF_ColorSetByConf                 " << "\t\t" << PARAM_TSDF_ColorSetByConf                    << std::endl;
        std::cout << "PARAM_TSDF_DepthTrunc_MaxPOS              " << "\t\t" << PARAM_TSDF_DepthTrunc_MaxPOS                 << std::endl;
        std::cout << "PARAM_TSDF_DepthTrunc_MaxNEG              " << "\t\t" << PARAM_TSDF_DepthTrunc_MaxNEG                 << std::endl;
        std::cout << "PARAM_TSDF_SensorDistBound_MIN            " << "\t\t" << PARAM_TSDF_SensorDistBound_MIN               << std::endl;
        std::cout << "PARAM_TSDF_SensorDistBound_MAX            " << "\t\t" << PARAM_TSDF_SensorDistBound_MAX               << std::endl;
        std::cout << "PARAM_CAM_Intrinsics_fx                   " << "\t\t" << PARAM_CAM_Intrinsics_fx                      << std::endl;
        std::cout << "PARAM_CAM_Intrinsics_fy                   " << "\t\t" << PARAM_CAM_Intrinsics_fy                      << std::endl;
        std::cout << "PARAM_CAM_Intrinsics_px                   " << "\t\t" << PARAM_CAM_Intrinsics_px                      << std::endl;
        std::cout << "PARAM_CAM_Intrinsics_py                   " << "\t\t" << PARAM_CAM_Intrinsics_py                      << std::endl;
        std::cout << "PARAM_TSDF_ISO_level                      " << "\t\t" << PARAM_TSDF_ISO_level                         << std::endl;
        std::cout << "PARAM_TSDF_PercExtendGrid                 " << "\t\t" << PARAM_TSDF_PercExtendGrid                    << std::endl;
        std::cout << "PARAM_TSDF_Min_Weight_MC                  " << "\t\t" << PARAM_TSDF_Min_Weight_MC                     << std::endl;
        std::cout << "PARAM_TSDF_Img_Width                      " << "\t\t" << PARAM_TSDF_Img_Width                         << std::endl;
        std::cout << "PARAM_TSDF_Img_Height                     " << "\t\t" << PARAM_TSDF_Img_Height                        << std::endl;
        std::cout << "PARAM_TSDF_MaxWeightTruncLimit            " << "\t\t" << PARAM_TSDF_MaxWeightTruncLimit               << std::endl;
        std::cout << "PARAM_TSDF_NumRandomSplts                 " << "\t\t" << PARAM_TSDF_NumRandomSplts                    << std::endl;
        std::cout << "PARAM_TSDF_MaxVoxelSize                   " << "\t\t" << PARAM_TSDF_MaxVoxelSize                      << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_THRESH_CONV_m2mm                    " << "\t\t" << PARAM_THRESH_CONV_m2mm                       << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_BackGround_RRR                      " << "\t\t" << PARAM_BackGround_RRR                         << std::endl;
        std::cout << "PARAM_BackGround_GGG                      " << "\t\t" << PARAM_BackGround_GGG                         << std::endl;
        std::cout << "PARAM_BackGround_BBB                      " << "\t\t" << PARAM_BackGround_BBB                         << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_Viewer_PNG_Size_WWW                 " << "\t\t" << PARAM_Viewer_PNG_Size_WWW                    << std::endl;
        std::cout << "PARAM_Viewer_PNG_Size_HHH                 " << "\t\t" << PARAM_Viewer_PNG_Size_HHH                    << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_SkinnDetector_THRESH                " << "\t\t" << PARAM_SkinnDetector_THRESH                   << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_KEYpt_min_scale                     " << "\t\t" << PARAM_KEYpt_min_scale                        << std::endl;
        std::cout << "PARAM_KEYpt_nr_octaves                    " << "\t\t" << PARAM_KEYpt_nr_octaves                       << std::endl;
        std::cout << "PARAM_KEYpt_nr_scales_per_octave          " << "\t\t" << PARAM_KEYpt_nr_scales_per_octave             << std::endl;
        std::cout << "PARAM_KEYpt_min_contrast                  " << "\t\t" << PARAM_KEYpt_min_contrast                     << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_SIFT_KEY_nFeatures                  " << "\t\t" << PARAM_SIFT_KEY_nFeatures                     << std::endl;
        std::cout << "PARAM_SIFT_KEY_nOctaveLayers              " << "\t\t" << PARAM_SIFT_KEY_nOctaveLayers                 << std::endl;
        std::cout << "PARAM_SIFT_KEY_contrastThreshold          " << "\t\t" << PARAM_SIFT_KEY_contrastThreshold             << std::endl;
        std::cout << "PARAM_SIFT_KEY_edgeThreshold              " << "\t\t" << PARAM_SIFT_KEY_edgeThreshold                 << std::endl;
        std::cout << "PARAM_SIFT_KEY_sigma                      " << "\t\t" << PARAM_SIFT_KEY_sigma                         << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_RMATCHER_Ratio                      " << "\t\t" << PARAM_RMATCHER_Ratio                         << std::endl;
        std::cout << "PARAM_RMATCHER_MinDistanceToEpipolar      " << "\t\t" << PARAM_RMATCHER_MinDistanceToEpipolar         << std::endl;
        std::cout << "PARAM_RMATCHER_ConfidenceLevel            " << "\t\t" << PARAM_RMATCHER_ConfidenceLevel               << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout << "PARAM_RUN_EXTRA_DBG_COUT                  " << "\t\t" << PARAM_RUN_EXTRA_DBG_COUT                     << std::endl;
        std::cout <<                                                                                                           std::endl;
        std::cout <<                                                                                                           std::endl;

        if (applyHANDs)
        {
        std::cout <<                                                                                                              std::endl;
        std::cout << "PARAM_SKIN_Dist_THRESH                    " << "\t\t" << touch->PARAM_SKIN_Dist_THRESH                   << std::endl;
        std::cout << "PARAM_SKIN_Dist_THRESH_INCR               " << "\t\t" << touch->PARAM_SKIN_Dist_THRESH_INCR              << std::endl;
        std::cout <<                                                                                                              std::endl;
        std::cout << "PARAM_SKIN_minTouchingFINGs               " << "\t\t" << touch->PARAM_SKIN_minTouchingFINGs              << std::endl;
        std::cout << "PARAM_SKIN_minTouchingPointsPerFinger     " << "\t\t" << touch->PARAM_SKIN_minTouchingPointsPerFinger    << std::endl;
        std::cout <<                                                                                                              std::endl;
        std::cout << "PARAM_should_ONLY_EF                      " << "\t\t" << touch->PARAM_should_ONLY_EF                     << std::endl;
        std::cout <<                                                                                                              std::endl;
        }

        if (applyHANDs)
        {
        std::cout <<                                                                                                                                           std::endl;
        std::cout << "PARAM_SKIN_ENFORCE_MIN_TouchFING_2_EXIT         " << "\t\t" << touch->PARAM_SKIN_ENFORCE_MIN_TouchFING_2_EXIT                         << std::endl;
        std::cout << "PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger        " << "\t\t" << touch->PARAM_SKIN_ENFORCE_MIN_TouchPtsPerFinger                        << std::endl;
        std::cout << "PARAM_TOUCH_Mode__intersection_OR_union_OR_full " << "\t\t" << touch->PARAM_TOUCH_Mode__intersection_OR_union_OR_full.toStdString()   << std::endl;
        std::cout <<                                                                                                                                           std::endl;
        std::cout << "PARAM_ICP_Mode__previous_OR_metascan            " << "\t\t" <<        PARAM_ICP_Mode__previous_OR_metascan.     toStdString()         << std::endl;
        std::cout << "PARAM_ICP_Mode__p2p_OR_p2plane                  " << "\t\t" <<        PARAM_ICP_Mode__p2p_OR_p2plane.           toStdString()         << std::endl;
        std::cout << "PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION       " << "\t\t" <<        PARAM_ICP_Mode__p2p_OR_p2plane_TRANSITION.toStdString()         << std::endl;
        std::cout << "PARAM_OBJ_Mode__p2p_OR_p2plane                  " << "\t\t" <<        PARAM_OBJ_Mode__p2p_OR_p2plane.           toStdString()         << std::endl;
        }

}








void Registrator::read_CONFIG_PATHs()
{

        ///////////////////////////////////
        libconfig::Config configFile_PATHs;
        ///////////////////////////////////

        std::cout << PATH_CONFIG_PATHs.toStdString() << std::endl;

        try
        {
            configFile_PATHs.readFile( PATH_CONFIG_PATHs.toLatin1() );
        }
        catch(const libconfig::FileIOException &fioex)   {   std::cout << "Could not read config file "                                                    << std::endl;   return;   }
        catch(      libconfig::ParseException  &pex  )   {   std::cout << "Parse error at config file, Line: " << pex.getLine() << " - " << pex.getError() << std::endl;   return;   }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            VIDEO_FRAMES___BasePath   = QString::fromLatin1((const char*)configFile_PATHs.lookup( "Directory_where_FRAMES_zips_are_unzipped" ));
            PATH_OUTPUT_BASE          = QString::fromLatin1((const char*)configFile_PATHs.lookup( "Directory_where_MODELS_MOTION_CAMERA_zips_are_unzipped" ));
        ////////////////////////////////////////////
        if (PATH_OUTPUT_BASE.endsWith("/") == false)
            PATH_OUTPUT_BASE +=       "/";
        ///////////////////////////////////////////////////
        if (VIDEO_FRAMES___BasePath.endsWith("/") == false)
            VIDEO_FRAMES___BasePath +=       "/";
        ///////////////////////////////////////////////////

}

void Registrator::update_PATHs( int seqID )
{

        QString strrr_SEQ = QString::number( seqID ).rightJustified(3,'0',false);


                                                    PATH_MODELS_INFO  = VIDEO_FRAMES___BasePath + strrr_SEQ + "/" + "MODELS_INFO.txt";
                                                    PATH_INDEX_BOUNDS = VIDEO_FRAMES___BasePath + strrr_SEQ + "/" + "INDEX_BOUNDS.txt";
        if (syntheticORrealistic == "synthetic")    PATH_CAMERA_SET   = PATH_OUTPUT_BASE        + strrr_SEQ + "/" + "Cameras.txt";
        else                                        PATH_CAMERA_SET   = PATH_OUTPUT_BASE        + strrr_SEQ + "/" + "Cameras.txt";

}

void Registrator::update_PATHs_print()
{

        std::cout <<                                                                                                   std::endl;
        std::cout << "******************************************"                                                   << std::endl;
        std::cout << "**** HAS HANDS  **************************"                                                   << std::endl;
        std::cout << "******************************************"                                                   << std::endl;
        std::cout <<                                                                                                   std::endl;
        std::cout << "PATH_MODELS_INFO                          " << "\t\t" << PATH_MODELS_INFO. toStdString()      << std::endl;
        std::cout << "PATH_INDEX_BOUNDS                         " << "\t\t" << PATH_INDEX_BOUNDS.toStdString()      << std::endl;
        std::cout << "PATH_CAMERA_SET                           " << "\t\t" << PATH_CAMERA_SET.  toStdString()      << std::endl;
        std::cout << "PATH_OUTPUT_BASE                          " << "\t\t" << PATH_OUTPUT_BASE. toStdString()      << std::endl;
        std::cout <<                                                                                                   std::endl;

}



