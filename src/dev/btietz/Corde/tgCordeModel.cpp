#include "tgCordeModel.h"

#include "dev/btietz/Corde/CordeModel.h"


tgCordeModel::tgCordeModel()
{

}
    
tgCordeModel::~tgCordeModel()
{
    
}
    
void tgCordeModel::setup(tgWorld& world)
{
        #if (1) // X Pos
	btVector3 startPos(0.0, 0.0, 0.0);
	btVector3 endPos  (10.0, 0.0, 0.0);
	
	// Setup for neither bending nor rotation
	btQuaternion startRot( 0, sqrt(2)/2.0, 0, sqrt(2)/2.0);
	btQuaternion endRot = startRot;
#else
	btVector3 startPos(0.0, 0.0, 0.0);
	btVector3 endPos  (0.0, 0.0, 10.0);
	
	// Setup for neither bending nor rotation
	btQuaternion startRot( 0, 0, 0, 1);
	btQuaternion endRot = startRot;
#endif	
	// Values for Rope from Spillman's paper
	const std::size_t resolution = 10;
	const double radius = 0.01;
	const double density = 1300;
	const double youngMod = 0.5;
	const double shearMod = 0.5;
	const double stretchMod = 20.0;
	const double springConst = 100.0 * pow(10, 3);
	const double gammaT = 10.0 * pow(10, -6);
	const double gammaR = 1.0 * pow(10, -6);
	CordeModel::Config config(resolution, radius, density, youngMod, shearMod,
								stretchMod, springConst, gammaT, gammaR);
	
	testString = new CordeModel(startPos, endPos, startRot, endRot, config);
}

void tgCordeModel::teardown()
{
    delete testString;
}
    
void tgCordeModel::step(double dt)
{
    testString.step(dt);
}
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void tgCordeModel::onVisit(const tgModelVisitor& r) const
{

}
