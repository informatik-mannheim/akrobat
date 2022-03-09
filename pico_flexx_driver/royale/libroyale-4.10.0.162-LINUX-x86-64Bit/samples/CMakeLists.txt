#****************************************************************************
# Copyright (C) 2018 pmdtechnologies ag & Infineon Technologies
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# PARTICULAR PURPOSE.
#
#****************************************************************************

cmake_minimum_required(VERSION 3.13)

project (royale_samples)

# C++ samples
add_subdirectory(cpp/sampleCameraInfo)
add_subdirectory(cpp/sampleRetrieveData)
add_subdirectory(cpp/sampleIReplay)
add_subdirectory(cpp/sampleOpenCV)
add_subdirectory(cpp/sampleSlave)
add_subdirectory(cpp/sampleExportPLY)
add_subdirectory(cpp/sampleQtViewer)
add_subdirectory(cpp/sampleRecordRRF)
add_subdirectory(cpp/sampleMultiCamera)

