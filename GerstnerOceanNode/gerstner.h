//
//  gerstner.h
//  GerstnerOceanNode
//
//  Created by Steven Dao on 7/24/13.
//
//

#ifndef GerstnerOceanNode_gerstner_h
#define GerstnerOceanNode_gerstner_h

class gerstner
{
public:
    gerstner(const double time,
             const double steepness[3],
             const double amplitude[3],
             const double length[3],
             const double speed[3],
             const MVector direction[3]);
	virtual 		~gerstner() {};
    MPoint     simulate(const MPoint original);
private:
    double          amplitude[3];
    double          w[3];       // frequency
    double          phi[3];     // phase shift
    MVector         wd[3];      // w * direction
    double          pt[3];      // phi * time
    MVector         sad[3];     // speed * amplitude * direction
};

#endif
