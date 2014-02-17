//
//  gerstner.h
//  GerstnerOceanNode
//
//  Created by Steven Dao on 7/23/13.
//
//  Based on the algorithms presented in Tessendorf (2001) and Finch (GPU Gems, 2004).
//

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include "gerstner.h"

#define GRAVITY 9.8 // m/s^2

/***************************
 * Gerstner wave functions *
 ***************************/

/*!
 * Calculates the frequency (little-omega) for waves in deep water, given the wavelength L of the wave.
 */
static double w(const double L)
{
    return sqrt(GRAVITY * 2 * M_PI / L);
}

/*!
 * Calculates the phase-constant (phi) for a specified speed S and wavelength L.
 */
static double phi(const double S, const double L)
{
    return S * 2 * M_PI / L;
}

/*!
 * Creates a new Gerstner simulation by pre-calculating data that is needed for the Gerstner calculations at a specific time.
 *
 * Arguments:
 *    original -- the original point
 *    time -- the time of the simulation
 *    steepness -- 0..1 where 0 is rolling sine wave and 1 is the sharpest crest possible
 *    amplitude -- the amplitude of the wave
 *    length -- the length of the wave from crest to crest
 *    direction -- the horizontal direction in which the wave travels (direction.y == 0; ||direction|| == 1)
 *    speed - the speed at which the crest of the wave moves in the specified direction
 */
gerstner::gerstner(const double time, const double steepness[3], const double amplitude[3], const double length[3], const double speed[3], const MVector direction[3])
{
    for (int i = 0; i < 3; i++) {
        gerstner::amplitude[i] = amplitude[i];
        gerstner::w[i] = ::w(length[i]);
        gerstner::phi[i] = ::phi(speed[i], length[i]);
        gerstner::wd[i] = gerstner::w[i] * direction[i];
        gerstner::pt[i] = gerstner::phi[i] * time;
        gerstner::sad[i] = steepness[i] * amplitude[i] * direction[i];
    }
}

/*!
 * Calculates the wave-displaced point for a given point using the Gerstner wave functions.
 */
MPoint gerstner::simulate(const MPoint original)
{
    MPoint output(original.x, 0, original.z);
    
    for ( int i = 0; i < 3; i++ ) {
        double wd_dot_x0_plus_pt = gerstner::wd[i] * original + gerstner::pt[i];
        
        output += gerstner::sad[i] * cos(wd_dot_x0_plus_pt);
        output.y += gerstner::amplitude[i] * sin(wd_dot_x0_plus_pt);
    }
    
    return output;
}
