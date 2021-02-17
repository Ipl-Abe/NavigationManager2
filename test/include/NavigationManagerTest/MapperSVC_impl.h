// -*-C++-*-
/*!
 * @file  MapperSVC_impl.h
 * @brief Service implementation header of Mapper.idl
 *
 */

#include "NavigationDataTypeSkel.h"
#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "MapperSkel.h"

#ifndef MAPPERSVC_IMPL_H
#define MAPPERSVC_IMPL_H
 
/*!
 * @class NAVIGATION_OccupancyGridMapperSVC_impl
 * Example class implementing IDL interface NAVIGATION::OccupancyGridMapper
 */
class NAVIGATION_OccupancyGridMapperSVC_impl
 : public virtual POA_NAVIGATION::OccupancyGridMapper,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~NAVIGATION_OccupancyGridMapperSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   NAVIGATION_OccupancyGridMapperSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~NAVIGATION_OccupancyGridMapperSVC_impl();

   // attributes and operations
   NAVIGATION::MAPPER_RETURN_VALUE initializeMap(const NAVIGATION::OccupancyGridMapConfig& config, const RTC::Pose2D& initialPose);
   NAVIGATION::MAPPER_RETURN_VALUE startMapping();
   NAVIGATION::MAPPER_RETURN_VALUE stopMapping();
   NAVIGATION::MAPPER_RETURN_VALUE suspendMapping();
   NAVIGATION::MAPPER_RETURN_VALUE resumeMapping();
   NAVIGATION::MAPPER_RETURN_VALUE getState(NAVIGATION::MAPPER_STATE& state);

};



#endif // MAPPERSVC_IMPL_H


