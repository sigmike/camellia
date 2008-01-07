# Microsoft Developer Studio Project File - Name="camellia" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=camellia - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "camellia.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "camellia.mak" CFG="camellia - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "camellia - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "camellia - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "camellia - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /G6 /MD /W3 /GX /Zi /O2 /Ob2 /I "inc" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /FD /Zm1000 /O3 -QaxW /c
# ADD BASE RSC /l 0x40c /d "NDEBUG"
# ADD RSC /l 0x40c /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"lib\camellia.lib"

!ELSEIF  "$(CFG)" == "camellia - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /I "inc" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /D "CAM_DEBUG" /FR /FD /Zm1000 /GZ /c
# ADD BASE RSC /l 0x40c /d "_DEBUG"
# ADD RSC /l 0x40c /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"lib\camelliad.lib"

!ENDIF 

# Begin Target

# Name "camellia - Win32 Release"
# Name "camellia - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\src\cam_3d.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_3d_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_arithmetics.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_arithmetics_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_blob_analysis.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_capture.cpp
# End Source File
# Begin Source File

SOURCE=.\src\cam_draw.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_draw_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_error.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_harris.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_histogram.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_hls.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_hls_hypot_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_hls_phasearg_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_hls_sqrt_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_hough.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_hvsumming.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_integralimage.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_io.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_keypoints.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_keypoints_matching.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_keypoints_sectors_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_labelling.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_linear_filtering.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_linear_filtering_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_LUT.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_LUT_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_ME.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_measures.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_measures_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_median_filtering.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_median_filtering_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths_code_circle5.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths_code_circle7.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths_code_opt.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_morphomaths_code_square3.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_labelling.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_labelling_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_labelling_table1.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_labelling_table2.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_morpho.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_RLE_utils.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_SAD.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_separable_filter_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_utils.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_utils_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_volberg.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_warping.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_warping_code.c
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=.\src\cam_watershed.c
# End Source File
# Begin Source File

SOURCE=.\src\cam_yuv.c
# End Source File
# Begin Source File

SOURCE=.\src\camellia.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\inc\camellia.h
# End Source File
# Begin Source File

SOURCE=.\inc\camellia_internals.h
# End Source File
# End Group
# End Target
# End Project
