// -*-C++-*-
/*!
 * @file  PathPlannerSVC_impl.h
 * @brief Service implementation header of PathPlanner.idl
 *
 */

#include "NavigationDataTypeSkel.h"
#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "PathPlannerSkel.h"

#ifndef PATHPLANNERSVC_IMPL_H
#define PATHPLANNERSVC_IMPL_H
 
/*!
 * @class NAVIGATION_PathPlannerSVC_impl
 * Example class implementing IDL interface NAVIGATION::PathPlanner
 */
class NAVIGATION_PathPlannerSVC_impl
 : public virtual POA_NAVIGATION::PathPlanner,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~NAVIGATION_PathPlannerSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   NAVIGATION_PathPlannerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~NAVIGATION_PathPlannerSVC_impl();

   // attributes and operations
   NAVIGATION::PLAN_RETURN_STATUS planPath(const NAVIGATION::OccupancyGridMap& map, const NAVIGATION::PathPlanParameter& param, NAVIGATION::Path2D_out outPath);

};



#endif // PATHPLANNERSVC_IMPL_H


