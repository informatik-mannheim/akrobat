/****************************************************************************\
* Copyright (C) 2019 pmdtechnologies ag
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#pragma once

#include<royale/DepthImage.hpp>
#include<royale/IRImage.hpp>

namespace royale
{
    /**
     *  This represents combination of both depth and IR image.
     *  Provides depth, confidence and IR 8Bit mono information for every pixel.
     */
    struct DepthIRImage
    {
        int64_t                   timestamp;       //!< timestamp for the frame
        StreamId                  streamId;        //!< stream which produced the data
        uint16_t                  width;           //!< width of depth image
        uint16_t                  height;          //!< height of depth image
        royale::Vector<uint16_t>  dpData;          //!< depth and confidence for the pixel
        royale::Vector<uint8_t>   irData;          //!< 8Bit mono IR image
    };
}
