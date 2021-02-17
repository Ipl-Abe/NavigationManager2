// -*-C++-*-
/*!
 * @file  PathPlannerSVC_impl.cpp
 * @brief Service implementation code of PathPlanner.idl
 *
 */

#include "PathPlannerSVC_impl.h"

/*
 * Example implementational code for IDL interface NAVIGATION::PathPlanner
 */
NAVIGATION_PathPlannerSVC_impl::NAVIGATION_PathPlannerSVC_impl()
{
  // Please add extra constructor code here.
}


NAVIGATION_PathPlannerSVC_impl::~NAVIGATION_PathPlannerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
NAVIGATION::PLAN_RETURN_STATUS NAVIGATION_PathPlannerSVC_impl::planPath(const NAVIGATION::OccupancyGridMap& map, const NAVIGATION::PathPlanParameter& param, NAVIGATION::Path2D_out outPath)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::PLAN_RETURN_STATUS NAVIGATION_PathPlannerSVC_impl::planPath(const NAVIGATION::OccupancyGridMap& map, const NAVIGATION::PathPlanParameter& param, NAVIGATION::Path2D_out outPath)>"
#endif
  return 0;
}



// End of example implementational code



