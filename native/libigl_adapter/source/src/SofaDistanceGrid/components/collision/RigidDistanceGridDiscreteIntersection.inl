/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_INL
#define SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_INL
#include <iostream>
#include <algorithm>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/Intersection.inl>

#include "RigidDistanceGridDiscreteIntersection.h"
#include "HybridExactContact.h"
#include <stdexcept>

namespace sofa
{

namespace component
{

namespace collision
{

template<class T>
bool RigidDistanceGridDiscreteIntersection::testIntersection(RigidDistanceGridCollisionElement&, geometry::TSphere<T>&, const core::collision::Intersection*)
{
    return true;
}

template<class T>
int RigidDistanceGridDiscreteIntersection::computeIntersection(RigidDistanceGridCollisionElement& e1, geometry::TSphere<T>& e2, OutputVector* contacts, const core::collision::Intersection* intersection)
{
    DistanceGrid* grid1 = e1.getGrid();
    const auto* model=e1.getCollisionModel();
    if(model->getSize()!=1 || model->isMoving() || model->isSimulated())
        throw std::runtime_error("Hybrid requires one static non-simulated wall element");
    const type::Vec3& t1=e1.getTranslation();const sofa::type::Matrix3&r1=e1.getRotation();
    for(int k=0;k<3;k++){
        if(std::abs(t1[k])>1e-12)throw std::runtime_error("Hybrid requires identity wall translation");
        for(int j=0;j<3;j++)if(std::abs(r1[k][j]-(k==j?1.:0.))>1e-12)throw std::runtime_error("Hybrid requires identity wall rotation");
    }
    if(std::abs(intersection->getContactDistance())>1e-15)
        throw std::runtime_error("Hybrid requires DiscreteIntersection contactDistance=0; wall proximity carries .2mm offset");
    campaign_hybrid::ExactContact contact;
    if(!campaign_hybrid::query(grid1,model,model->fileRigidDistanceGrid.getFullPath(),e2.center(),e2.r(),e1.getProximity(),e2.getProximity(),contact))return 0;
    const double d0=e1.getProximity()+e2.getProximity()+intersection->getContactDistance()+e2.r();
    contacts->resize(contacts->size()+1);core::collision::DetectionOutput*detection=&*(contacts->end()-1);
    detection->normal=contact.normal;detection->value=contact.signedDistance-d0;
    detection->elem.first=e1;detection->elem.second=e2;detection->id=e2.getIndex();
    detection->point[0]=contact.point;
    detection->point[1]=e2.getContactPointByNormal(detection->normal);
    return 1;
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
