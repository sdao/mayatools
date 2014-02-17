//
//  helpers.h
//  GerstnerOceanNode
//
//  Created by Steven Dao on 7/24/13.
//
//

#ifndef GerstnerOceanNode_helpers
#define GerstnerOceanNode_helpers

/*************************
 * Math helper functions *
 *************************/

static double sinD(const double deg)
{
    return sin(deg * M_PI / 180.);
}

static double cosD(const double deg)
{
    return cos(deg * M_PI / 180.);
}

#endif
