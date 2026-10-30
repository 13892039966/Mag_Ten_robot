#pragma once
#include <SofaDistanceGrid/DistanceGrid.h>
#include <sofa/type/Vec.h>
#include <string>
namespace campaign_hybrid {
struct ExactContact {sofa::type::Vec3 point,normal;double signedDistance;};
bool query(sofa::component::container::DistanceGrid* grid,const void* model,const std::string&rawPath,
           const sofa::type::Vec3& p,double radius,double wallProximity,double sphereProximity,ExactContact& out);
}
