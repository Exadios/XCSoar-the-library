set(XCSOAR ${XCSoar-the-library_SOURCE_DIR})
set(XCSOAR_SRC ${XCSOAR}/src)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)

include_directories(${XCSOAR_SRC})
add_definitions(-DZLIB_CONST) # Make some of zlib const.
add_definitions(-DUSE_WGS84 -DRADIANS -DEYE_CANDY)

set(UTIL_DIR ${XCSOAR_SRC}/Util)
set(UTIL_SRCS ${UTIL_DIR}/Exception.cxx
              ${UTIL_DIR}/PrintException.cxx
              ${UTIL_DIR}/Base64.cxx
              ${UTIL_DIR}/CRC.cpp
              ${UTIL_DIR}/ASCII.cxx
              ${UTIL_DIR}/TruncateString.cpp
              ${UTIL_DIR}/EscapeBackslash.cpp
              ${UTIL_DIR}/StringUtil.cpp
              ${UTIL_DIR}/ConvertString.cpp
              ${UTIL_DIR}/StaticString.cxx
              ${UTIL_DIR}/AllocatedString.cxx
              ${UTIL_DIR}/StringView.cxx
              ${UTIL_DIR}/StringCompare.cxx
              ${UTIL_DIR}/ExtractParameters.cpp
              ${UTIL_DIR}/UTF8.cpp
              ${UTIL_DIR}/tstring.cpp)
set(UTIL_HDRS ${UTIL_DIR}/Algorithm.hpp
              ${UTIL_DIR}/CastIterator.hpp
              ${UTIL_DIR}/Clamp.hpp
              ${UTIL_DIR}/ConvertString.hpp
              ${UTIL_DIR}/CRC.hpp
              ${UTIL_DIR}/DereferenceIterator.hpp
              ${UTIL_DIR}/DollarExpand.hpp
              ${UTIL_DIR}/EnumBitSet.hpp
              ${UTIL_DIR}/EnumCast.hpp
              ${UTIL_DIR}/EscapeBackslash.hpp
              ${UTIL_DIR}/ExtractParameters.hpp
              ${UTIL_DIR}/GlobalSliceAllocator.hpp
              ${UTIL_DIR}/Macros.hpp
              ${UTIL_DIR}/NonCopyable.hpp
              ${UTIL_DIR}/NumberParser.hpp
              ${UTIL_DIR}/OverwritingRingBuffer.hpp
              ${UTIL_DIR}/QuadTree.hpp
              ${UTIL_DIR}/RadixTree.hpp
              ${UTIL_DIR}/Range.hpp
              ${UTIL_DIR}/ReservablePriorityQueue.hpp
              ${UTIL_DIR}/ReusableArray.hpp
              ${UTIL_DIR}/Serial.hpp
              ${UTIL_DIR}/SliceAllocator.hpp
              ${UTIL_DIR}/StringFormat.hpp
              ${UTIL_DIR}/StringUtil.hpp
              ${UTIL_DIR}/TriState.hpp
              ${UTIL_DIR}/TruncateString.hpp
              ${UTIL_DIR}/tstring.hpp
              ${UTIL_DIR}/TypeTraits.hpp
              ${UTIL_DIR}/UTF8.hpp
              ${UTIL_DIR}/WStringFormat.hpp
              ${UTIL_DIR}/WStringUtil.hpp
              ${UTIL_DIR}/AllocatedArray.hxx
              ${UTIL_DIR}/AllocatedGrid.hxx
              ${UTIL_DIR}/AllocatedString.hxx
              ${UTIL_DIR}/ASCII.hxx
              ${UTIL_DIR}/Base64.hxx
              ${UTIL_DIR}/Cache.hxx
              ${UTIL_DIR}/CharUtil.hxx
              ${UTIL_DIR}/ConstBuffer.hxx
              ${UTIL_DIR}/ContainerCast.hxx
              ${UTIL_DIR}/DeleteDisposer.hxx
              ${UTIL_DIR}/DynamicFifoBuffer.hxx
              ${UTIL_DIR}/Exception.hxx
              ${UTIL_DIR}/ForeignFifoBuffer.hxx
              ${UTIL_DIR}/IterableSplitString.hxx
              ${UTIL_DIR}/LightString.hxx
              ${UTIL_DIR}/Manual.hxx
              ${UTIL_DIR}/PrintException.hxx
              ${UTIL_DIR}/ScopeExit.hxx
              ${UTIL_DIR}/StaticArray.hxx
              ${UTIL_DIR}/StaticFifoBuffer.hxx
              ${UTIL_DIR}/StaticString.hxx
              ${UTIL_DIR}/StringAPI.hxx
              ${UTIL_DIR}/StringBuffer.hxx
              ${UTIL_DIR}/StringBuilder.hxx
              ${UTIL_DIR}/StringCompare.hxx
              ${UTIL_DIR}/StringParser.hxx
              ${UTIL_DIR}/StringPointer.hxx
              ${UTIL_DIR}/StringView.hxx
              ${UTIL_DIR}/TextFile.hxx
              ${UTIL_DIR}/TrivialArray.hxx
              ${UTIL_DIR}/TStringView.hxx
              ${UTIL_DIR}/WASCII.hxx
              ${UTIL_DIR}/WCharUtil.hxx
              ${UTIL_DIR}/WritableBuffer.hxx
              ${UTIL_DIR}/WStringAPI.hxx
              ${UTIL_DIR}/WStringCompare.hxx
              ${UTIL_DIR}/WStringView.hxx)
include_directories(${XCSOAR_SRC} ${XCSOAR_SRC}/unix ${UTIL_DIR})
add_library(Util-static-${T} STATIC ${UTIL_SRCS})
add_library(Util-shared-${T} SHARED ${UTIL_SRCS})
add_library(Util-object-${T} OBJECT ${UTIL_SRCS})
install(FILES ${UTIL_HDRS}   DESTINATION "include/Util")

set(GEO_DIR ${XCSOAR_SRC}/Geo)
set(GEO_SRCS ${GEO_DIR}/ConvexHull/GrahamScan.cpp
	           ${GEO_DIR}/ConvexHull/PolygonInterior.cpp
	           ${GEO_DIR}/Memento/DistanceMemento.cpp
	           ${GEO_DIR}/Memento/GeoVectorMemento.cpp
	           ${GEO_DIR}/Flat/FlatProjection.cpp
	           ${GEO_DIR}/Flat/TaskProjection.cpp
	           ${GEO_DIR}/Flat/FlatBoundingBox.cpp
	           ${GEO_DIR}/Flat/FlatGeoPoint.cpp
	           ${GEO_DIR}/Flat/FlatRay.cpp
	           ${GEO_DIR}/Flat/FlatPoint.cpp
	           ${GEO_DIR}/Flat/FlatEllipse.cpp
	           ${GEO_DIR}/Flat/FlatLine.cpp
	           ${GEO_DIR}/Math.cpp
	           ${GEO_DIR}/SimplifiedMath.cpp
	           ${GEO_DIR}/GeoPoint.cpp
	           ${GEO_DIR}/GeoVector.cpp
	           ${GEO_DIR}/GeoBounds.cpp
	           ${GEO_DIR}/GeoClip.cpp
	           ${GEO_DIR}/SearchPoint.cpp
	           ${GEO_DIR}/SearchPointVector.cpp
	           ${GEO_DIR}/GeoEllipse.cpp
	           ${GEO_DIR}/UTM.cpp)
set(GEO_HDRS ${GEO_DIR}/AltitudeReference.hpp
             ${GEO_DIR}/CoordinateFormat.hpp
             ${GEO_DIR}/FAISphere.hpp
             ${GEO_DIR}/GeoBounds.hpp
             ${GEO_DIR}/GeoClip.hpp
             ${GEO_DIR}/GeoEllipse.hpp
             ${GEO_DIR}/Geoid.hpp
             ${GEO_DIR}/GeoPoint.hpp
             ${GEO_DIR}/GeoVector.hpp
             ${GEO_DIR}/Gravity.hpp
             ${GEO_DIR}/Math.hpp
             ${GEO_DIR}/Quadrilateral.hpp
             ${GEO_DIR}/SearchPoint.hpp
             ${GEO_DIR}/SearchPointVector.hpp
             ${GEO_DIR}/SimplifiedMath.hpp
             ${GEO_DIR}/SpeedVector.hpp
             ${GEO_DIR}/UTM.hpp
             ${GEO_DIR}/WGS84.hpp)
include_directories(${XCSOAR_SRC} ${GEO_DIR})
add_library(Geo-static-${T} STATIC ${GEO_SRCS})
add_library(Geo-shared-${T} SHARED ${GEO_SRCS})
add_library(Geo-object-${T} OBJECT ${GEO_SRCS})
install(FILES ${GEO_HDRS}   DESTINATION "include/Geo")

set(MATH_DIR ${XCSOAR_SRC}/Math)
set(MATH_SRCS ${MATH_DIR}/Angle.cpp
              ${MATH_DIR}/ARange.cpp
              ${MATH_DIR}/FastMath.cpp
              ${MATH_DIR}/FastTrig.cpp
              ${MATH_DIR}/FastRotation.cpp
              ${MATH_DIR}/LeastSquares.cpp
              ${MATH_DIR}/DiffFilter.cpp
              ${MATH_DIR}/Filter.cpp
              ${MATH_DIR}/ZeroFinder.cpp
              ${MATH_DIR}/KalmanFilter1d.cpp
              ${MATH_DIR}/SelfTimingKalmanFilter1d.cpp)
set(MATH_HDRS ${MATH_DIR}/Angle.hpp
              ${MATH_DIR}/ARange.hpp
              ${MATH_DIR}/AvFilter.hpp
              ${MATH_DIR}/Constants.hpp
              ${MATH_DIR}/ConvexFilter.hpp
              ${MATH_DIR}/DifferentialWindowFilter.hpp
              ${MATH_DIR}/DiffFilter.hpp
              ${MATH_DIR}/FastMath.hpp
              ${MATH_DIR}/FastRotation.hpp
              ${MATH_DIR}/FastTrig.hpp
              ${MATH_DIR}/Filter.hpp
              ${MATH_DIR}/Histogram.hpp
              ${MATH_DIR}/KalmanFilter1d.hpp
              ${MATH_DIR}/LeastSquares.hpp
              ${MATH_DIR}/Line2D.hpp
              ${MATH_DIR}/LowPassFilter.hpp
              ${MATH_DIR}/Point2D.hpp
              ${MATH_DIR}/Quadratic.hpp
              ${MATH_DIR}/Quadrilateral.hpp
              ${MATH_DIR}/Screen.hpp
              ${MATH_DIR}/SelfTimingKalmanFilter1d.hpp
              ${MATH_DIR}/Shift.hpp
              ${MATH_DIR}/SunEphemeris.hpp
              ${MATH_DIR}/Trig.hpp
              ${MATH_DIR}/Util.hpp
              ${MATH_DIR}/Vector.hpp
              ${MATH_DIR}/WindowFilter.hpp
              ${MATH_DIR}/XYDataStore.hpp
              ${MATH_DIR}/ZeroFinder.hpp)
set(MATH_BOOST_HDRS ${MATH_DIR}/Boost/Point.hpp)
include_directories(${XCSOAR_SRC} ${MATH_DIR} ${OUTPUT_INCLUDE})
add_library(Math-static-${T} STATIC ${MATH_SRCS})
add_library(Math-shared-${T} SHARED ${MATH_SRCS})
add_library(Math-object-${T} OBJECT ${MATH_SRCS})
install(FILES ${MATH_BOOST_HDRS} DESTINATION "include/Math/Boost")
install(FILES ${MATH_HDRS}       DESTINATION "include/Math")

set(IO_DIR ${XCSOAR_SRC}/IO)
set(IO_SRCS ${IO_DIR}/BufferedReader.cxx
            ${IO_DIR}/FileReader.cxx
            ${IO_DIR}/BufferedOutputStream.cxx
            ${IO_DIR}/FileOutputStream.cxx
            ${IO_DIR}/GunzipReader.cxx
            ${IO_DIR}/ZlibError.cxx
            ${IO_DIR}/FileTransaction.cpp
            ${IO_DIR}/FileCache.cpp
            ${IO_DIR}/ZipArchive.cpp
            ${IO_DIR}/ZipArchive.cpp
            ${IO_DIR}/ZipReader.cpp
            ${IO_DIR}/ConvertLineReader.cpp
            ${IO_DIR}/FileLineReader.cpp
            ${IO_DIR}/KeyValueFileReader.cpp
            ${IO_DIR}/KeyValueFileWriter.cpp
            ${IO_DIR}/ZipLineReader.cpp
            ${IO_DIR}/CSVLine.cpp
            ${IO_DIR}/TextWriter.cpp
            ${IO_DIR}/MapFile.cpp
            ${IO_DIR}/ConfiguredFile.cpp
            ${IO_DIR}/DataFile.cpp)
set(IO_HDRS ${IO_DIR}/Charset.hpp
            ${IO_DIR}/ConfiguredFile.hpp
            ${IO_DIR}/ConvertLineReader.hpp
            ${IO_DIR}/CSVLine.hpp
            ${IO_DIR}/DataFile.hpp
            ${IO_DIR}/DataHandler.hpp
            ${IO_DIR}/FileCache.hpp
            ${IO_DIR}/FileHandle.hpp
            ${IO_DIR}/FileLineReader.hpp
            ${IO_DIR}/FileTransaction.hpp
            ${IO_DIR}/KeyValueFileReader.hpp
            ${IO_DIR}/KeyValueFileWriter.hpp
            ${IO_DIR}/LineReader.hpp
            ${IO_DIR}/MapFile.hpp
            ${IO_DIR}/NullDataHandler.hpp
            ${IO_DIR}/TextWriter.hpp
            ${IO_DIR}/ZipArchive.hpp
            ${IO_DIR}/ZipLineReader.hpp
            ${IO_DIR}/ZipReader.hpp
            ${IO_DIR}/BufferedOutputStream.hxx
            ${IO_DIR}/BufferedReader.hxx
            ${IO_DIR}/FileOutputStream.hxx
            ${IO_DIR}/FileReader.hxx
            ${IO_DIR}/GunzipReader.hxx
            ${IO_DIR}/OutputStream.hxx
            ${IO_DIR}/Reader.hxx
            ${IO_DIR}/StdioOutputStream.hxx
            ${IO_DIR}/ZlibError.hxx)
set(IO_ASYNC_HDRS ${IO_DIR}/Async/AsioThread.hpp
                  ${IO_DIR}/Async/AsioUtil.hpp
                  ${IO_DIR}/Async/GlobalAsioThread.hpp
                  ${IO_DIR}/Async/SignalListener.hpp)
include_directories(${XCSOAR_SRC} ${IO_DIR})
add_library(Io-static-${T} STATIC ${IO_SRCS})
add_library(Io-shared-${T} SHARED ${IO_SRCS})
add_library(Io-object-${T} OBJECT ${IO_SRCS})
install(FILES ${IO_ASYNC_HDRS} DESTINATION "include/IO/Async")
install(FILES ${IO_HDRS}       DESTINATION "include/IO")

set(WAYPOINTENGINE_DIR ${XCSOAR_SRC}/Engine/Waypoint)
set(WAYPOINTENGINE_SRCS ${WAYPOINTENGINE_DIR}/WaypointVisitor.cpp
                        ${WAYPOINTENGINE_DIR}/Waypoints.cpp
                        ${WAYPOINTENGINE_DIR}/Waypoint.cpp)
set(WAYPOINTENGINE_HDRS ${WAYPOINTENGINE_DIR}/Origin.hpp
                        ${WAYPOINTENGINE_DIR}/Ptr.hpp
                        ${WAYPOINTENGINE_DIR}/Runway.hpp
                        ${WAYPOINTENGINE_DIR}/Waypoint.hpp
                        ${WAYPOINTENGINE_DIR}/Waypoints.hpp
                        ${WAYPOINTENGINE_DIR}/WaypointVisitor.hpp)
include_directories(${XCSOAR_SRC} ${WAYPOINTENGINE_DIR})
add_library(WaypointEngine-static-${T} STATIC ${WAYPOINTENGINE_SRCS})
add_library(WaypointEngine-shared-${T} SHARED ${WAYPOINTENGINE_SRCS})
add_library(WaypointEngine-object-${T} OBJECT ${WAYPOINTENGINE_SRCS})
target_link_libraries(WaypointEngine-static-${T} Geo-static-${T} Math-static-${T})
target_link_libraries(WaypointEngine-shared-${T} Geo-shared-${T} Math-shared-${T})
install(FILES ${WAYPOINTENGINE_HDRS} DESTINATION "include/Engine/Waypoint")

set(ROUTEENGINE_DIR ${XCSOAR_SRC}/Engine/Route)
set(ROUTEENGINE_SRCS ${ROUTEENGINE_DIR}/Config.cpp
                     ${ROUTEENGINE_DIR}/RoutePlanner.cpp
                     ${ROUTEENGINE_DIR}/AirspaceRoute.cpp
                     ${ROUTEENGINE_DIR}/TerrainRoute.cpp
                     ${ROUTEENGINE_DIR}/RouteLink.cpp
                     ${ROUTEENGINE_DIR}/RoutePolar.cpp
                     ${ROUTEENGINE_DIR}/RoutePolars.cpp
                     ${ROUTEENGINE_DIR}/FlatTriangleFan.cpp
                     ${ROUTEENGINE_DIR}/FlatTriangleFanTree.cpp
                     ${ROUTEENGINE_DIR}/ReachFan.cpp
                     ${ROUTEENGINE_DIR}/Config.cpp)
set(ROUTEENGINE_HDRS ${ROUTEENGINE_DIR}/AirspaceRoute.hpp
                     ${ROUTEENGINE_DIR}/AStar.hpp
                     ${ROUTEENGINE_DIR}/Config.hpp
                     ${ROUTEENGINE_DIR}/FlatTriangleFan.hpp
                     ${ROUTEENGINE_DIR}/FlatTriangleFanTree.hpp
                     ${ROUTEENGINE_DIR}/Point.hpp
                     ${ROUTEENGINE_DIR}/ReachFan.hpp
                     ${ROUTEENGINE_DIR}/ReachFanParms.hpp
                     ${ROUTEENGINE_DIR}/ReachResult.hpp
                     ${ROUTEENGINE_DIR}/Route.hpp
                     ${ROUTEENGINE_DIR}/RouteLink.hpp
                     ${ROUTEENGINE_DIR}/RoutePlanner.hpp
                     ${ROUTEENGINE_DIR}/RoutePolar.hpp
                     ${ROUTEENGINE_DIR}/RoutePolars.hpp
                     ${ROUTEENGINE_DIR}/TerrainRoute.hpp)
include_directories(${XCSOAR_SRC} ${ROUTEENGINE_DIR} ${XCSOAR_SRC}/Engine)
add_library(RouteEngine-static-${T} STATIC ${ROUTEENGINE_SRCS})
add_library(RouteEngine-shared-${T} SHARED ${ROUTEENGINE_SRCS})
add_library(RouteEngine-object-${T} OBJECT ${ROUTEENGINE_SRCS})
install(FILES ${ROUTEENGINE_HDRS}    DESTINATION "include/Engine/Route")

set(GLIDEENGINE_DIR ${XCSOAR_SRC}/Engine/GlideSolvers)
set(GLIDEENGINE_SRCS ${GLIDEENGINE_DIR}/GlideSettings.cpp
                     ${GLIDEENGINE_DIR}/GlideState.cpp
                     ${GLIDEENGINE_DIR}/GlueGlideState.cpp
                     ${GLIDEENGINE_DIR}/GlidePolar.cpp
                     ${GLIDEENGINE_DIR}/PolarCoefficients.cpp
                     ${GLIDEENGINE_DIR}/GlideResult.cpp
                     ${GLIDEENGINE_DIR}/MacCready.cpp
                     ${GLIDEENGINE_DIR}/InstantSpeed.cpp)
set(GLIDEENGINE_HDRS ${GLIDEENGINE_DIR}/GlidePolar.hpp
                     ${GLIDEENGINE_DIR}/GlideResult.hpp
                     ${GLIDEENGINE_DIR}/GlideSettings.hpp
                     ${GLIDEENGINE_DIR}/GlideState.hpp
                     ${GLIDEENGINE_DIR}/MacCready.hpp
                     ${GLIDEENGINE_DIR}/PolarCoefficients.hpp)
include_directories(${XCSOAR_SRC} ${GLIDEENGINE_DIR})
add_library(GlideEngine-static-${T} STATIC ${GLIDEENGINE_SRCS})
add_library(GlideEngine-shared-${T} SHARED ${GLIDEENGINE_SRCS})
add_library(GlideEngine-object-${T} OBJECT ${GLIDEENGINE_SRCS})
install(FILES ${GLIDEENGINE_HDRS} DESTINATION "include/Engine/GlideSolvers")

set(CONTESTENGINE_DIR ${XCSOAR_SRC}/Engine/Contest)
set(CONTESTENGINE_SRCS ${CONTESTENGINE_DIR}/Settings.cpp
                       ${CONTESTENGINE_DIR}/ContestManager.cpp
                       ${CONTESTENGINE_DIR}/Solvers/Contests.cpp
                       ${CONTESTENGINE_DIR}/Solvers/AbstractContest.cpp
                       ${CONTESTENGINE_DIR}/Solvers/TraceManager.cpp
                       ${CONTESTENGINE_DIR}/Solvers/ContestDijkstra.cpp
                       ${CONTESTENGINE_DIR}/Solvers/DMStQuad.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCLeague.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCSprint.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCClassic.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCTriangle.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCFAI.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCPlus.cpp
                       ${CONTESTENGINE_DIR}/Solvers/DMStQuad.cpp
                       ${CONTESTENGINE_DIR}/Solvers/XContestFree.cpp
                       ${CONTESTENGINE_DIR}/Solvers/XContestTriangle.cpp
                       ${CONTESTENGINE_DIR}/Solvers/OLCSISAT.cpp
                       ${CONTESTENGINE_DIR}/Solvers/NetCoupe.cpp
                       ${CONTESTENGINE_DIR}/Solvers/Retrospective.cpp)
set(CONTESTENGINE_HDRS ${CONTESTENGINE_DIR}/ContestManager.hpp
                       ${CONTESTENGINE_DIR}/ContestResult.hpp
                       ${CONTESTENGINE_DIR}/ContestStatistics.hpp
                       ${CONTESTENGINE_DIR}/ContestTrace.hpp
                       ${CONTESTENGINE_DIR}/Settings.hpp)
include_directories(${XCSOAR_SRC} ${CONTESTENGINE_DIR})
add_library(ContestEngine-static-${T} STATIC ${CONTESTENGINE_SRCS})
add_library(ContestEngine-shared-${T} SHARED ${CONTESTENGINE_SRCS})
add_library(ContestEngine-object-${T} OBJECT ${CONTESTENGINE_SRCS})
install(FILES ${CONTESTENGINE_HDRS} DESTINATION "include/Engine/Contest")

set(TASKENGINE_DIR ${XCSOAR_SRC}/Engine/Task)
set(TASKENGINE_SRCS ${TASKENGINE_DIR}/Shapes/FAITriangleSettings.cpp
                    ${TASKENGINE_DIR}/Shapes/FAITriangleRules.cpp
                    ${TASKENGINE_DIR}/Shapes/FAITriangleArea.cpp
                    ${TASKENGINE_DIR}/Shapes/FAITriangleTask.cpp
                    ${TASKENGINE_DIR}/Shapes/FAITrianglePointValidator.cpp
                    ${TASKENGINE_DIR}/TaskBehaviour.cpp
                    ${TASKENGINE_DIR}/TaskManager.cpp
                    ${TASKENGINE_DIR}/AbstractTask.cpp
                    ${TASKENGINE_DIR}/Ordered/StartConstraints.cpp
                    ${TASKENGINE_DIR}/Ordered/FinishConstraints.cpp
                    ${TASKENGINE_DIR}/Ordered/Settings.cpp
                    ${TASKENGINE_DIR}/Ordered/OrderedTask.cpp
                    ${TASKENGINE_DIR}/Ordered/TaskAdvance.cpp
                    ${TASKENGINE_DIR}/Ordered/SmartTaskAdvance.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/IntermediatePoint.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/OrderedTaskPoint.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/StartPoint.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/FinishPoint.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/ASTPoint.cpp
                    ${TASKENGINE_DIR}/Ordered/Points/AATPoint.cpp
                    ${TASKENGINE_DIR}/Ordered/AATIsoline.cpp
                    ${TASKENGINE_DIR}/Ordered/AATIsolineSegment.cpp
                    ${TASKENGINE_DIR}/Unordered/UnorderedTask.cpp
                    ${TASKENGINE_DIR}/Unordered/UnorderedTaskPoint.cpp
                    ${TASKENGINE_DIR}/Unordered/GotoTask.cpp
                    ${TASKENGINE_DIR}/Unordered/AbortTask.cpp
                    ${TASKENGINE_DIR}/Unordered/AlternateTask.cpp
                    ${TASKENGINE_DIR}/Factory/AbstractTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/RTTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/FAITaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/FAITriangleTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/FAIORTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/FAIGoalTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/AATTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/MatTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/MixedTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/TouringTaskFactory.cpp
                    ${TASKENGINE_DIR}/Factory/Create.cpp
                    ${TASKENGINE_DIR}/Points/TaskPoint.cpp
                    ${TASKENGINE_DIR}/Points/SampledTaskPoint.cpp
                    ${TASKENGINE_DIR}/Points/ScoredTaskPoint.cpp
                    ${TASKENGINE_DIR}/Points/TaskLeg.cpp
                    ${TASKENGINE_DIR}/ObservationZones/Boundary.cpp
                    ${TASKENGINE_DIR}/ObservationZones/ObservationZoneClient.cpp
                    ${TASKENGINE_DIR}/ObservationZones/ObservationZonePoint.cpp
                    ${TASKENGINE_DIR}/ObservationZones/CylinderZone.cpp
                    ${TASKENGINE_DIR}/ObservationZones/SectorZone.cpp
                    ${TASKENGINE_DIR}/ObservationZones/LineSectorZone.cpp
                    ${TASKENGINE_DIR}/ObservationZones/SymmetricSectorZone.cpp
                    ${TASKENGINE_DIR}/ObservationZones/KeyholeZone.cpp
                    ${TASKENGINE_DIR}/ObservationZones/AnnularSectorZone.cpp
                    ${TASKENGINE_DIR}/PathSolvers/TaskDijkstra.cpp
                    ${TASKENGINE_DIR}/PathSolvers/TaskDijkstraMin.cpp
                    ${TASKENGINE_DIR}/PathSolvers/TaskDijkstraMax.cpp
                    ${TASKENGINE_DIR}/PathSolvers/IsolineCrossingFinder.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskMacCready.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskMacCreadyTravelled.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskMacCreadyRemaining.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskMacCreadyTotal.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskBestMc.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskSolveTravelled.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskCruiseEfficiency.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskEffectiveMacCready.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskMinTarget.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskOptTarget.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskGlideRequired.cpp
                    ${TASKENGINE_DIR}/Solvers/TaskSolution.cpp
                    ${TASKENGINE_DIR}/Computer/ElementStatComputer.cpp
                    ${TASKENGINE_DIR}/Computer/DistanceStatComputer.cpp
                    ${TASKENGINE_DIR}/Computer/IncrementalSpeedComputer.cpp
                    ${TASKENGINE_DIR}/Computer/TaskVarioComputer.cpp
                    ${TASKENGINE_DIR}/Computer/TaskStatsComputer.cpp
                    ${TASKENGINE_DIR}/Computer/WindowStatsComputer.cpp
                    ${TASKENGINE_DIR}/Stats/CommonStats.cpp
                    ${TASKENGINE_DIR}/Stats/ElementStat.cpp
                    ${TASKENGINE_DIR}/Stats/TaskStats.cpp
                    ${TASKENGINE_DIR}/Stats/StartStats.cpp)
set(TASKENGINE_HDRS ${TASKENGINE_DIR}/AbstractTask.hpp
                    ${TASKENGINE_DIR}/TaskBehaviour.hpp
                    ${TASKENGINE_DIR}/TaskEvents.hpp
                    ${TASKENGINE_DIR}/TaskInterface.hpp
                    ${TASKENGINE_DIR}/TaskManager.hpp
                    ${TASKENGINE_DIR}/TaskType.hpp)
include_directories(${XCSOAR_SRC} ${TASKENGINE_DIR})
add_library(TaskEngine-static-${T} STATIC ${TASKENGINE_SRCS})
add_library(TaskEngine-shared-${T} SHARED ${TASKENGINE_SRCS})
add_library(TaskEngine-object-${T} OBJECT ${TASKENGINE_SRCS})
# TaskEngine should not depend on XCSoarMain but is made necessary because of
# the sematics of the original XCSoar build system.
target_link_libraries(TaskEngine-static-${T} GlideEngine-static-${T} 
                                             RouteEngine-static-${T}
                                             XCSoarMain-static-${T})
target_link_libraries(TaskEngine-shared-${T} GlideEngine-shared-${T} 
                                             RouteEngine-shared-${T}
                                             XCSoarMain-shared-${T})
install(FILES ${TASKENGINE_HDRS} DESTINATION "include/Engine/Task")

set(AIRSPACEENGINE_DIR ${XCSOAR_SRC}/Engine/Airspace)
set(ENGINE_DIR   ${XCSOAR_SRC}/Engine)
set(AIRSPACEENGINE_SRCS ${ENGINE_DIR}/Util/AircraftStateFilter.cpp
                        ${AIRSPACEENGINE_DIR}/AirspacesTerrain.cpp
                        ${AIRSPACEENGINE_DIR}/Airspace.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceAltitude.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceAircraftPerformance.cpp
                        ${AIRSPACEENGINE_DIR}/AbstractAirspace.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceCircle.cpp
                        ${AIRSPACEENGINE_DIR}/AirspacePolygon.cpp
                        ${AIRSPACEENGINE_DIR}/Airspaces.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceIntersectSort.cpp
                        ${AIRSPACEENGINE_DIR}/SoonestAirspace.cpp
                        ${AIRSPACEENGINE_DIR}/Predicate/AirspacePredicate.cpp
                        ${AIRSPACEENGINE_DIR}/Predicate/AirspacePredicateHeightRange.cpp
                        ${AIRSPACEENGINE_DIR}/Predicate/OutsideAirspacePredicate.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceIntersectionVisitor.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarningConfig.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarningManager.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarning.cpp
                        ${AIRSPACEENGINE_DIR}/AirspaceSorter.cpp)
set(AIRSPACEENGINE_HDRS ${AIRSPACEENGINE_DIR}/AbstractAirspace.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceActivity.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceAircraftPerformance.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceAltitude.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceCircle.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceClass.hpp
                        ${AIRSPACEENGINE_DIR}/Airspace.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceInterceptSolution.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceIntersectionVector.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceIntersectionVisitor.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceIntersectSort.hpp
                        ${AIRSPACEENGINE_DIR}/AirspacePolygon.hpp
                        ${AIRSPACEENGINE_DIR}/Airspaces.hpp
                        ${AIRSPACEENGINE_DIR}/AirspacesInterface.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceSorter.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceVisitor.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarningConfig.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarning.hpp
                        ${AIRSPACEENGINE_DIR}/AirspaceWarningManager.hpp
                        ${AIRSPACEENGINE_DIR}/Minimum.hpp
                        ${AIRSPACEENGINE_DIR}/SoonestAirspace.hpp)
include_directories(${XCSOAR_SRC} ${AIRSPACEENGINE_DIR})
add_library(AirspaceEngine-static-${T} STATIC ${AIRSPACEENGINE_SRCS})
add_library(AirspaceEngine-shared-${T} SHARED ${AIRSPACEENGINE_SRCS})
add_library(AirspaceEngine-object-${T} OBJECT ${AIRSPACEENGINE_SRCS})
install(FILES ${AIRSPACEENGINE_HDRS} DESTINATION "include/Engine/Airspace")

# Device drivers
set(DRIVER_DIR ${XCSOAR_SRC}/Device/Driver)
set(VOLKSLOGGER_SRCS ${DRIVER_DIR}/Volkslogger/Register.cpp
                     ${DRIVER_DIR}/Volkslogger/Parser.cpp
                     ${DRIVER_DIR}/Volkslogger/Protocol.cpp
                     ${DRIVER_DIR}/Volkslogger/Declare.cpp
                     ${DRIVER_DIR}/Volkslogger/Database.cpp
                     ${DRIVER_DIR}/Volkslogger/Util.cpp
                     ${DRIVER_DIR}/Volkslogger/dbbconv.cpp
                     ${DRIVER_DIR}/Volkslogger/grecord.cpp
                     ${DRIVER_DIR}/Volkslogger/vlapi2.cpp
                     ${DRIVER_DIR}/Volkslogger/vlapihlp.cpp
                     ${DRIVER_DIR}/Volkslogger/vlconv.cpp
                     ${DRIVER_DIR}/Volkslogger/Logger.cpp)
set(VOLKSLOGGER_HDRS ${DRIVER_DIR}/Volkslogger/Database.hpp
                     ${DRIVER_DIR}/Volkslogger/Internal.hpp
                     ${DRIVER_DIR}/Volkslogger/Protocol.hpp
                     ${DRIVER_DIR}/Volkslogger/Util.hpp)
set(CAI302_SRCS ${DRIVER_DIR}/CAI302/Protocol.cpp
                ${DRIVER_DIR}/CAI302/PocketNav.cpp
                ${DRIVER_DIR}/CAI302/Mode.cpp
                ${DRIVER_DIR}/CAI302/Parser.cpp
                ${DRIVER_DIR}/CAI302/Settings.cpp
                ${DRIVER_DIR}/CAI302/Declare.cpp
                ${DRIVER_DIR}/CAI302/Logger.cpp
                ${DRIVER_DIR}/CAI302/Manage.cpp
                ${DRIVER_DIR}/CAI302/Register.cpp)
set(CAI302_HDRS ${DRIVER_DIR}/CAI302/Internal.hpp
                ${DRIVER_DIR}/CAI302/PocketNav.hpp
                ${DRIVER_DIR}/CAI302/Protocol.hpp)
set(IMI_SRCS ${DRIVER_DIR}/IMI/Protocol/MessageParser.cpp
             ${DRIVER_DIR}/IMI/Protocol/Communication.cpp
             ${DRIVER_DIR}/IMI/Protocol/Checksum.cpp
             ${DRIVER_DIR}/IMI/Protocol/Conversion.cpp
             ${DRIVER_DIR}/IMI/Protocol/IGC.cpp
             ${DRIVER_DIR}/IMI/Protocol/Protocol.cpp
             ${DRIVER_DIR}/IMI/Declare.cpp
             ${DRIVER_DIR}/IMI/Internal.cpp
             ${DRIVER_DIR}/IMI/Logger.cpp
             ${DRIVER_DIR}/IMI/Register.cpp)
set(IMI_PROTOCOL_HDRS ${DRIVER_DIR}/IMI/Protocol/Checksum.hpp
                      ${DRIVER_DIR}/IMI/Protocol/Communication.hpp
                      ${DRIVER_DIR}/IMI/Protocol/Conversion.hpp
                      ${DRIVER_DIR}/IMI/Protocol/IGC.hpp
                      ${DRIVER_DIR}/IMI/Protocol/MessageParser.hpp
                      ${DRIVER_DIR}/IMI/Protocol/Protocol.hpp
                      ${DRIVER_DIR}/IMI/Protocol/Types.hpp)
set(IMI_HDRS ${DRIVER_DIR}/IMI/Internal.hpp)
set(LX_SRCS ${DRIVER_DIR}/LX/NanoLogger.cpp
            ${DRIVER_DIR}/LX/NanoDeclare.cpp
            ${DRIVER_DIR}/LX/Protocol.cpp
            ${DRIVER_DIR}/LX/Mode.cpp
            ${DRIVER_DIR}/LX/Parser.cpp
            ${DRIVER_DIR}/LX/Settings.cpp
            ${DRIVER_DIR}/LX/Declare.cpp
            ${DRIVER_DIR}/LX/Logger.cpp
            ${DRIVER_DIR}/LX/Convert.cpp
            ${DRIVER_DIR}/LX/LXN.cpp
            ${DRIVER_DIR}/LX/Register.cpp)
set(LX_HDRS ${DRIVER_DIR}/LX/Convert.hpp
            ${DRIVER_DIR}/LX/Internal.hpp
            ${DRIVER_DIR}/LX/LX1600.hpp
            ${DRIVER_DIR}/LX/LXN.hpp
            ${DRIVER_DIR}/LX/NanoDeclare.hpp
            ${DRIVER_DIR}/LX/NanoLogger.hpp
            ${DRIVER_DIR}/LX/NanoProtocol.hpp
            ${DRIVER_DIR}/LX/Protocol.hpp
            ${DRIVER_DIR}/LX/V7.hpp)
set(FLARM_SRCS ${DRIVER_DIR}/FLARM/Device.cpp
               ${DRIVER_DIR}/FLARM/Register.cpp
               ${DRIVER_DIR}/FLARM/Mode.cpp
               ${DRIVER_DIR}/FLARM/Parser.cpp
               ${DRIVER_DIR}/FLARM/StaticParser.cpp
               ${DRIVER_DIR}/FLARM/Settings.cpp
               ${DRIVER_DIR}/FLARM/Declare.cpp
               ${DRIVER_DIR}/FLARM/Logger.cpp
               ${DRIVER_DIR}/FLARM/CRC16.cpp
               ${DRIVER_DIR}/FLARM/BinaryProtocol.cpp
               ${DRIVER_DIR}/FLARM/TextProtocol.cpp)
set(FLARM_HDRS ${DRIVER_DIR}/FLARM/BinaryProtocol.hpp
               ${DRIVER_DIR}/FLARM/CRC16.hpp
               ${DRIVER_DIR}/FLARM/Device.hpp
               ${DRIVER_DIR}/FLARM/StaticParser.hpp
               ${DRIVER_DIR}/FLARM/TextProtocol.hpp)
set(FLYTEC_SRCS ${DRIVER_DIR}/Flytec/Register.cpp
                ${DRIVER_DIR}/Flytec/Parser.cpp
                ${DRIVER_DIR}/Flytec/Logger.cpp)
set(FLYTEC_HDRS ${DRIVER_DIR}/Flytec/Device.hpp)
set(VEGA_SRCS ${DRIVER_DIR}/Vega/Misc.cpp
              ${DRIVER_DIR}/Vega/Parser.cpp
              ${DRIVER_DIR}/Vega/Settings.cpp
              ${DRIVER_DIR}/Vega/Volatile.cpp
              ${DRIVER_DIR}/Vega/Register.cpp)
set(VEGA_HDRS ${DRIVER_DIR}/Vega/Internal.hpp
              ${DRIVER_DIR}/Vega/Volatile.hpp)
set(BLUEFLY_SRCS ${DRIVER_DIR}/BlueFly/Misc.cpp
                 ${DRIVER_DIR}/BlueFly/Parser.cpp
                 ${DRIVER_DIR}/BlueFly/Settings.cpp
                 ${DRIVER_DIR}/BlueFly/Register.cpp)
set(BLUEFLY_HDRS ${DRIVER_DIR}/BlueFly/Internal.hpp)
set(OTHER_SRCS ${DRIVER_DIR}/AltairPro.cpp
               ${DRIVER_DIR}/BorgeltB50.cpp
               ${DRIVER_DIR}/CaiGpsNav.cpp
               ${DRIVER_DIR}/CaiLNav.cpp
               ${DRIVER_DIR}/Condor.cpp
               ${DRIVER_DIR}/CProbe.cpp
               ${DRIVER_DIR}/EW.cpp
               ${DRIVER_DIR}/EWMicroRecorder.cpp
               ${DRIVER_DIR}/Eye.cpp
               ${DRIVER_DIR}/FlymasterF1.cpp
               ${DRIVER_DIR}/FlyNet.cpp
               ${DRIVER_DIR}/Generic.cpp
               ${DRIVER_DIR}/LevilAHRS_G.cpp
               ${DRIVER_DIR}/Leonardo.cpp
               ${DRIVER_DIR}/NmeaOut.cpp
               ${DRIVER_DIR}/OpenVario.cpp
               ${DRIVER_DIR}/PosiGraph.cpp
               ${DRIVER_DIR}/XCOM760.cpp
               ${DRIVER_DIR}/ILEC.cpp
               ${DRIVER_DIR}/Westerboer.cpp
               ${DRIVER_DIR}/Zander.cpp
               ${DRIVER_DIR}/Vaulter.cpp
               ${DRIVER_DIR}/ATR833.cpp)
set(OTHER_HDRS ${DRIVER_DIR}/Eye.hpp
               ${DRIVER_DIR}/FLARM.hpp
               ${DRIVER_DIR}/FlymasterF1.hpp
               ${DRIVER_DIR}/FlyNet.hpp
               ${DRIVER_DIR}/Flytec.hpp
               ${DRIVER_DIR}/Generic.hpp
               ${DRIVER_DIR}/ILEC.hpp
               ${DRIVER_DIR}/IMI.hpp
               ${DRIVER_DIR}/KRT2.hpp
               ${DRIVER_DIR}/Leonardo.hpp
               ${DRIVER_DIR}/LevilAHRS_G.hpp
               ${DRIVER_DIR}/LX.hpp
               ${DRIVER_DIR}/NmeaOut.hpp
               ${DRIVER_DIR}/OpenVario.hpp
               ${DRIVER_DIR}/PosiGraph.hpp
               ${DRIVER_DIR}/ThermalExpress.hpp
               ${DRIVER_DIR}/Vaulter.hpp
               ${DRIVER_DIR}/Vega.hpp
               ${DRIVER_DIR}/Volkslogger.hpp
               ${DRIVER_DIR}/Westerboer.hpp
               ${DRIVER_DIR}/XCOM760.hpp
               ${DRIVER_DIR}/XCTracer.hpp
               ${DRIVER_DIR}/Zander.hpp)
set(DRIVER_SRCS ${VOLKSLOGGER_SRCS}
                ${CAI302_SRCS}
                ${IMI_SRCS}
                ${LX_SRCS}
                ${FLARM_SRCS}
                ${FLYTEC_SRCS}
                ${VEGA_SRCS}
                ${BLUEFLY_SRCS}
                ${OTHER_SRCS})
set(DRIVER_HDRS ${VOLKSLOGGER_HDRS}
                ${CAI302_HDRS}
                ${IMI_HDRS}
                ${LX_HDRS}
                ${FLARM_HDRS}
                ${FLYTEC_HDRS}
                ${VEGA_HDRS}
                ${BLUEFLY_HDRS}
                ${OTHER_HDRS})
include_directories(${XCSOAR_SRC} ${DRIVER_DIR})
add_library(Driver-static-${T} STATIC ${DRIVER_SRCS})
add_library(Driver-shared-${T} SHARED ${DRIVER_SRCS})
add_library(Driver-object-${T} OBJECT ${DRIVER_SRCS})
install(FILES ${VOLKSLOGGER_HDRS} DESTINATION "include/Device/Driver/Volkslogger")
install(FILES ${CAI302_HDRS}      DESTINATION "include/Device/Driver/CAI302")
install(FILES ${IMI_HDRS}         DESTINATION "include/Device/Driver/IMI")
install(FILES ${LX_HDRS}          DESTINATION "include/Device/Driver/LX")
install(FILES ${FLARM_HDRS}       DESTINATION "include/Device/Driver/FLARM")
install(FILES ${FLYTEC_HDRS}      DESTINATION "include/Device/Driver/Flytec")
install(FILES ${VEGA_HDRS}        DESTINATION "include/Device/Driver/Vega")
install(FILES ${BLUEFLY_HDRS}     DESTINATION "include/Device/Driver/BlueFly")
install(FILES ${OTHER_HDRS}       DESTINATION "include/Device/Driver")

set(SHAPE_DIR ${XCSOAR_SRC}/Topography/shapelib)
set(SHAPE_SRCS ${SHAPE_DIR}/mapalloc.c
               ${SHAPE_DIR}/mapstring.c
               ${SHAPE_DIR}/mapbits.c
               ${SHAPE_DIR}/mapprimitive.c
               ${SHAPE_DIR}/mapsearch.c
               ${SHAPE_DIR}/mapshape.c
               ${SHAPE_DIR}/maptree.c
               ${SHAPE_DIR}/mapxbase.c)
set(SHAPE_HDRS ${SHAPE_DIR}/maperror.h
               ${SHAPE_DIR}/mapprimitive.h
               ${SHAPE_DIR}/mapserver-config.h
               ${SHAPE_DIR}/mapserver.h
               ${SHAPE_DIR}/mapshape.h
               ${SHAPE_DIR}/maptree.h)
include_directories(${XCSOAR_SRC} ${SHAPE_DIR})
add_library(Shape-static-${T} STATIC ${SHAPE_SRCS})
add_library(Shape-shared-${T} SHARED ${SHAPE_SRCS})
add_library(Shape-object-${T} OBJECT ${SHAPE_SRCS})
set_target_properties(Shape-object-${T}
                      PROPERTIES COMPILE_FLAGS "-fPIC")
install(FILES ${SHAPE_HDRS} DESTINATION "include/Topography/shapelib")

set(OS_DIR ${XCSOAR_SRC}/OS)
set(OS_SRCS ${OS_DIR}/Clock.cpp
            ${OS_DIR}/FileDescriptor.cxx
            ${OS_DIR}/FileMapping.cpp
            ${OS_DIR}/FileUtil.cpp
            ${OS_DIR}/RunFile.cpp
            ${OS_DIR}/Path.cpp
            ${OS_DIR}/PathName.cpp
            ${OS_DIR}/Process.cpp
            ${OS_DIR}/SystemLoad.cpp)
set(OS_HDRS ${OS_DIR}/Args.hpp
            ${OS_DIR}/ByteOrder.hpp
            ${OS_DIR}/Clock.hpp
            ${OS_DIR}/ConvertPathName.hpp
            ${OS_DIR}/DynamicLibrary.hpp
            ${OS_DIR}/EventPipe.hpp
            ${OS_DIR}/FileMapping.hpp
            ${OS_DIR}/FileUtil.hpp
            ${OS_DIR}/LogError.hpp
            ${OS_DIR}/OverlappedEvent.hpp
            ${OS_DIR}/Path.hpp
            ${OS_DIR}/PathName.hpp
            ${OS_DIR}/Process.hpp
            ${OS_DIR}/RunFile.hpp
            ${OS_DIR}/SystemLoad.hpp
            ${OS_DIR}/Error.hxx
            ${OS_DIR}/FileDescriptor.hxx
            ${OS_DIR}/UniqueFileDescriptor.hxx)
if(HAVE_POSIX)
  set(OS_SRCS ${OS_SRCS} ${OS_DIR}/EventPipe.cpp)
endif(HAVE_POSIX)
add_library(Os-static-${T} STATIC ${OS_SRCS})
add_library(Os-shared-${T} SHARED ${OS_SRCS})
add_library(Os-object-${T} OBJECT ${OS_SRCS})
install(FILES ${OS_HDRS} DESTINATION "include/OS")

set(THREAD_DIR ${XCSOAR_SRC}/Thread)
set(THREAD_SRCS	${THREAD_DIR}/Thread.cpp
                ${THREAD_DIR}/SuspensibleThread.cpp
                ${THREAD_DIR}/RecursivelySuspensibleThread.cpp
                ${THREAD_DIR}/WorkerThread.cpp
                ${THREAD_DIR}/StandbyThread.cpp
                ${THREAD_DIR}/Debug.cpp)
set(THREAD_HDRS ${THREAD_DIR}/Debug.hpp
                ${THREAD_DIR}/FastMutex.hpp
                ${THREAD_DIR}/Guard.hpp
                ${THREAD_DIR}/Handle.hpp
                ${THREAD_DIR}/Mutex.hpp
                ${THREAD_DIR}/Name.hpp
                ${THREAD_DIR}/RecursivelySuspensibleThread.hpp
                ${THREAD_DIR}/SharedMutex.hpp
                ${THREAD_DIR}/StandbyThread.hpp
                ${THREAD_DIR}/StoppableThread.hpp
                ${THREAD_DIR}/SuspensibleThread.hpp
                ${THREAD_DIR}/Thread.hpp
                ${THREAD_DIR}/Trigger.hpp
                ${THREAD_DIR}/Util.hpp
                ${THREAD_DIR}/WorkerThread.hpp
                ${THREAD_DIR}/Cond.hxx
                ${THREAD_DIR}/CriticalSection.hxx
                ${THREAD_DIR}/FastSharedMutex.hxx
                ${THREAD_DIR}/PosixCond.hxx
                ${THREAD_DIR}/PosixMutex.hxx
                ${THREAD_DIR}/PosixSharedMutex.hxx
                ${THREAD_DIR}/WindowsCond.hxx
                ${THREAD_DIR}/WindowsSharedMutex.hxx)
add_library(Thread-static-${T} STATIC ${THREAD_SRCS})
add_library(Thread-shared-${T} SHARED ${THREAD_SRCS})
add_library(Thread-object-${T} OBJECT ${THREAD_SRCS})
target_link_libraries(Thread-static-${T} pthread)
target_link_libraries(Thread-shared-${T} pthread)
install(FILES ${THREAD_HDRS} DESTINATION "include/Thread")

set(TERRAIN_DIR ${XCSOAR_SRC}/Terrain)
set(TERRAIN_SRCS ${TERRAIN_DIR}/RasterBuffer.cpp
                 ${TERRAIN_DIR}/RasterProjection.cpp
                 ${TERRAIN_DIR}/RasterMap.cpp
                 ${TERRAIN_DIR}/RasterTile.cpp
                 ${TERRAIN_DIR}/RasterTileCache.cpp
                 ${TERRAIN_DIR}/ZzipStream.cpp
                 ${TERRAIN_DIR}/Loader.cpp
                 ${TERRAIN_DIR}/WorldFile.cpp
                 ${TERRAIN_DIR}/Intersection.cpp
                 ${TERRAIN_DIR}/ScanLine.cpp
                 ${TERRAIN_DIR}/RasterTerrain.cpp
                 ${TERRAIN_DIR}/Thread.cpp
                 ${TERRAIN_DIR}/HeightMatrix.cpp
                 ${TERRAIN_DIR}/RasterRenderer.cpp
                 ${TERRAIN_DIR}/TerrainRenderer.cpp
                 ${TERRAIN_DIR}/TerrainSettings.cpp)
set(TERRAIN_HDRS ${TERRAIN_DIR}/Height.hpp
                 ${TERRAIN_DIR}/HeightMatrix.hpp
                 ${TERRAIN_DIR}/Loader.hpp
                 ${TERRAIN_DIR}/RasterBuffer.hpp
                 ${TERRAIN_DIR}/RasterLocation.hpp
                 ${TERRAIN_DIR}/RasterMap.hpp
                 ${TERRAIN_DIR}/RasterProjection.hpp
                 ${TERRAIN_DIR}/RasterRenderer.hpp
                 ${TERRAIN_DIR}/RasterTerrain.hpp
                 ${TERRAIN_DIR}/RasterTileCache.hpp
                 ${TERRAIN_DIR}/RasterTile.hpp
                 ${TERRAIN_DIR}/RasterTraits.hpp
                 ${TERRAIN_DIR}/TerrainRenderer.hpp
                 ${TERRAIN_DIR}/TerrainSettings.hpp
                 ${TERRAIN_DIR}/Thread.hpp
                 ${TERRAIN_DIR}/WorldFile.hpp
                 ${TERRAIN_DIR}/ZzipStream.hpp)
add_library(Terrain-static-${T} STATIC ${TERRAIN_SRCS})
add_library(Terrain-shared-${T} SHARED ${TERRAIN_SRCS})
add_library(Terrain-object-${T} OBJECT ${TERRAIN_SRCS})
target_link_libraries(Terrain-static-${T} Jasper-static-${T} pthread)
target_link_libraries(Terrain-shared-${T} Jasper-shared-${T} pthread)
install(FILES ${TERRAIN_HDRS} DESTINATION "include/Terrain")

set(PROFILE_DIR ${XCSOAR_SRC}/Profile)
set(PROFILE_SRCS ${PROFILE_DIR}/File.cpp
                 ${PROFILE_DIR}/Current.cpp
                 ${PROFILE_DIR}/Map.cpp
                 ${PROFILE_DIR}/StringValue.cpp
                 ${PROFILE_DIR}/NumericValue.cpp
                 ${PROFILE_DIR}/PathValue.cpp
                 ${PROFILE_DIR}/GeoValue.cpp
                 ${PROFILE_DIR}/ProfileKeys.cpp
                 ${PROFILE_DIR}/ProfileMap.cpp)
set(PROFILE_HDRS ${PROFILE_DIR}/AirspaceConfig.hpp
                 ${PROFILE_DIR}/ComputerProfile.hpp
                 ${PROFILE_DIR}/ContestProfile.hpp
                 ${PROFILE_DIR}/Current.hpp
                 ${PROFILE_DIR}/DeviceConfig.hpp
                 ${PROFILE_DIR}/File.hpp
                 ${PROFILE_DIR}/FlarmProfile.hpp
                 ${PROFILE_DIR}/InfoBoxConfig.hpp
                 ${PROFILE_DIR}/Map.hpp
                 ${PROFILE_DIR}/MapProfile.hpp
                 ${PROFILE_DIR}/PageProfile.hpp
                 ${PROFILE_DIR}/Profile.hpp
                 ${PROFILE_DIR}/ProfileKeys.hpp
                 ${PROFILE_DIR}/ProfileMap.hpp
                 ${PROFILE_DIR}/RouteProfile.hpp
                 ${PROFILE_DIR}/Settings.hpp
                 ${PROFILE_DIR}/SystemProfile.hpp
                 ${PROFILE_DIR}/TaskProfile.hpp
                 ${PROFILE_DIR}/TerrainConfig.hpp
                 ${PROFILE_DIR}/TrackingProfile.hpp
                 ${PROFILE_DIR}/UIProfile.hpp
                 ${PROFILE_DIR}/UnitsConfig.hpp
                 ${PROFILE_DIR}/WeatherProfile.hpp)
add_library(Profile-static-${T} STATIC ${PROFILE_SRCS})
add_library(Profile-shared-${T} SHARED ${PROFILE_SRCS})
add_library(Profile-object-${T} OBJECT ${PROFILE_SRCS})
target_link_libraries(Profile-static-${T} Io-static-${T})
target_link_libraries(Profile-shared-${T} Io-shared-${T})
install(FILES ${PROFILE_HDRS} DESTINATION "include/Profile")

set(TIME_DIR ${XCSOAR_SRC}/Time)
set(TIME_SRCS ${TIME_DIR}/DeltaTime.cpp
              ${TIME_DIR}/WrapClock.cpp
              ${TIME_DIR}/LocalTime.cpp
              ${TIME_DIR}/BrokenTime.cpp
              ${TIME_DIR}/BrokenDate.cpp
              ${TIME_DIR}/BrokenDateTime.cpp)
set(TIME_HDRS ${TIME_DIR}/BrokenDate.hpp
              ${TIME_DIR}/BrokenDateTime.hpp
              ${TIME_DIR}/BrokenTime.hpp
              ${TIME_DIR}/DateUtil.hpp
              ${TIME_DIR}/DeltaTime.hpp
              ${TIME_DIR}/GPSClock.hpp
              ${TIME_DIR}/LocalTime.hpp
              ${TIME_DIR}/PeriodClock.hpp
              ${TIME_DIR}/ReplayClock.hpp
              ${TIME_DIR}/RoughTime.hpp
              ${TIME_DIR}/TimeoutClock.hpp
              ${TIME_DIR}/WrapClock.hpp)
add_library(Time-static-${T} STATIC ${TIME_SRCS})
add_library(Time-shared-${T} SHARED ${TIME_SRCS})
add_library(Time-object-${T} OBJECT ${TIME_SRCS})
install(FILES ${TIME_HDRS} DESTINATION "include/Time")

set(SCREEN_DIR ${XCSOAR_SRC}/Screen)
set(SCREEN_SRCS ${SCREEN_DIR}/Debug.cpp
#                ${SCREEN_DIR}/Custom/Cache.cpp
#                ${SCREEN_DIR}/FreeType/Font.cpp
#                ${SCREEN_DIR}/FreeType/Init.cpp
   )
add_library(Screen-static-${T} STATIC ${SCREEN_SRCS})
add_library(Screen-shared-${T} SHARED ${SCREEN_SRCS})
add_library(Screen-object-${T} OBJECT ${SCREEN_SRCS})
target_link_libraries(Screen-static-${T} Util-static-${T} pthread freetype)
target_link_libraries(Screen-shared-${T} Util-shared-${T} pthread freetype)
target_include_directories(Screen-static-${T} SYSTEM PUBLIC
                           /usr/include/freetype2)  # For Font.cpp
target_include_directories(Screen-shared-${T} SYSTEM PUBLIC
                           /usr/include/freetype2)
# Do not bother to install the Screen library

set(JASPER_DIR ${XCSOAR_SRC}/Terrain/jasper)
set(JASPER_SRCS ${JASPER_DIR}/base/jas_malloc.c
                ${JASPER_DIR}/base/jas_seq.c
                ${JASPER_DIR}/base/jas_stream.c
                ${JASPER_DIR}/base/jas_string.c
                ${JASPER_DIR}/base/jas_tvp.c
                ${JASPER_DIR}/jp2/jp2_cod.c
                ${JASPER_DIR}/jpc/jpc_bs.c
                ${JASPER_DIR}/jpc/jpc_cs.c
                ${JASPER_DIR}/jpc/jpc_dec.c
                ${JASPER_DIR}/jpc/jpc_math.c
                ${JASPER_DIR}/jpc/jpc_mqdec.c
                ${JASPER_DIR}/jpc/jpc_mqcod.c
                ${JASPER_DIR}/jpc/jpc_qmfb.c
                ${JASPER_DIR}/jpc/jpc_rtc.cpp
                ${JASPER_DIR}/jpc/jpc_t1dec.c
                ${JASPER_DIR}/jpc/jpc_t1cod.c
                ${JASPER_DIR}/jpc/jpc_t2dec.c
                ${JASPER_DIR}/jpc/jpc_t2cod.c
                ${JASPER_DIR}/jpc/jpc_tagtree.c
                ${JASPER_DIR}/jpc/jpc_tsfb.c)
set(JASPER_JP2_HDRS ${JASPER_DIR}/jp2/jp2_cod.h
                    ${JASPER_DIR}/jp2/jp2_dec.h)
set(JASPER_JPC_HDRS ${JASPER_DIR}/jpc/jpc_bs.h
                    ${JASPER_DIR}/jpc/jpc_cod.h
                    ${JASPER_DIR}/jpc/jpc_cs.h
                    ${JASPER_DIR}/jpc/jpc_dec.h
                    ${JASPER_DIR}/jpc/jpc_enc.h
                    ${JASPER_DIR}/jpc/jpc_fix.h
                    ${JASPER_DIR}/jpc/jpc_flt.h
                    ${JASPER_DIR}/jpc/jpc_math.h
                    ${JASPER_DIR}/jpc/jpc_mct.h
                    ${JASPER_DIR}/jpc/jpc_mqcod.h
                    ${JASPER_DIR}/jpc/jpc_mqdec.h
                    ${JASPER_DIR}/jpc/jpc_mqenc.h
                    ${JASPER_DIR}/jpc/jpc_qmfb.h
                    ${JASPER_DIR}/jpc/jpc_t1cod.h
                    ${JASPER_DIR}/jpc/jpc_t1dec.h
                    ${JASPER_DIR}/jpc/jpc_t1enc.h
                    ${JASPER_DIR}/jpc/jpc_t2cod.h
                    ${JASPER_DIR}/jpc/jpc_t2dec.h
                    ${JASPER_DIR}/jpc/jpc_t2enc.h
                    ${JASPER_DIR}/jpc/jpc_tagtree.h
                    ${JASPER_DIR}/jpc/jpc_tsfb.h
                    ${JASPER_DIR}/jpc/jpc_util.h)
add_library(Jasper-static-${T} STATIC ${JASPER_SRCS})
add_library(Jasper-shared-${T} SHARED ${JASPER_SRCS})
add_library(Jasper-object-${T} OBJECT ${JASPER_SRCS})
set_target_properties(Jasper-object-${T}
                      PROPERTIES COMPILE_FLAGS "-fPIC")
#get_target_property(JASPER_INCLUDES Jasper-static-${T} INCLUDE_DIRECTORIES)
#set(JASPER_INCLUDES ${JASPER_INCLUDES} ${XCSOAR_SRC}/Terrain)
#set_target_properties(Jasper-static-${T}
#                      PROPERTIES INCLUDE_DIRECTORIES ${JASPER_INCLUDES})
include_directories(${XCSOAR_SRC}/Terrain)
# It is an unfortunate fact that Jasper compiles with warnings. Turn off
# the errors that would occur.
target_compile_options(Jasper-static-${T}
                       PRIVATE -Wno-error=implicit-function-declaration
                       PRIVATE -Wno-error=unused-but-set-parameter
                       PRIVATE -Wno-error=unused-but-set-variable
                       PRIVATE -Wno-error=type-limits
                       PRIVATE -Wno-error=sign-compare
                       PRIVATE -Wno-error=shift-negative-value
                       PRIVATE -Drestrict=__restrict__)
target_compile_options(Jasper-shared-${T}
                       PRIVATE -Wno-error=implicit-function-declaration
                       PRIVATE -Wno-error=unused-but-set-parameter
                       PRIVATE -Wno-error=unused-but-set-variable
                       PRIVATE -Wno-error=type-limits
                       PRIVATE -Wno-error=sign-compare
                       PRIVATE -Wno-error=shift-negative-value
                       PRIVATE -Drestrict=__restrict__)
target_compile_options(Jasper-object-${T}
                       PRIVATE -Wno-error=implicit-function-declaration
                       PRIVATE -Wno-error=unused-but-set-parameter
                       PRIVATE -Wno-error=unused-but-set-variable
                       PRIVATE -Wno-error=type-limits
                       PRIVATE -Wno-error=sign-compare
                       PRIVATE -Wno-error=shift-negative-value
                       PRIVATE -Drestrict=__restrict__)
target_link_libraries(Jasper-static-${T} Zzip-static-${T})
target_link_libraries(Jasper-shared-${T} Zzip-shared-${T})
install(FILES ${JASPER_JP2_HDRS} DESTINATION "include/Terrain/jasper/jp2")
install(FILES ${JASPER_JPC_HDRS} DESTINATION "include/Terrain/jasper/jpc")

set(ZZIP_DIR ${XCSOAR_SRC}/zzip)
set(ZZIP_SRCS ${ZZIP_DIR}/fetch.c
              ${ZZIP_DIR}/file.c
              ${ZZIP_DIR}/plugin.c
              ${ZZIP_DIR}/zip.c
              ${ZZIP_DIR}/stat.c)
set(ZZIP_HDRS ${ZZIP_DIR}/autoconf.h
              ${ZZIP_DIR}/conf.h
              ${ZZIP_DIR}/_config.h
              ${ZZIP_DIR}/__debug.h
              ${ZZIP_DIR}/fetch.h
              ${ZZIP_DIR}/file.h
              ${ZZIP_DIR}/format.h
              ${ZZIP_DIR}/__hints.h
              ${ZZIP_DIR}/info.h
              ${ZZIP_DIR}/lib.h
              ${ZZIP_DIR}/__mmap.h
              ${ZZIP_DIR}/plugin.h
              ${ZZIP_DIR}/stdint.h
              ${ZZIP_DIR}/types.h
              ${ZZIP_DIR}/util.h
              ${ZZIP_DIR}/zzip32.h
              ${ZZIP_DIR}/zzip.h)
add_library(Zzip-static-${T} STATIC ${ZZIP_SRCS})
add_library(Zzip-shared-${T} SHARED ${ZZIP_SRCS})
add_library(Zzip-object-${T} OBJECT ${ZZIP_SRCS})
target_link_libraries(Zzip-static-${T} z)
target_link_libraries(Zzip-shared-${T} z)
set_target_properties(Zzip-object-${T}
                      PROPERTIES COMPILE_FLAGS "-fPIC")
install(FILES ${ZZIP_HDRS} DESTINATION "include/zzip")

set(MAIN_SRCS ${XCSOAR_SRC}/LocalPath.cpp
              ${XCSOAR_SRC}/Profile/Profile.cpp
              ${XCSOAR_SRC}/LogFile.cpp
              ${XCSOAR_SRC}/Interface.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointList.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointListBuilder.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointFilter.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointGlue.cpp
              ${XCSOAR_SRC}/Task/DefaultTask.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointList.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointListBuilder.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointFilter.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointGlue.cpp
              ${XCSOAR_SRC}/Waypoint/SaveGlue.cpp
              ${XCSOAR_SRC}/Waypoint/LastUsed.cpp
              ${XCSOAR_SRC}/Waypoint/HomeGlue.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointFileType.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReader.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderBase.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderOzi.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderFS.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderWinPilot.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderSeeYou.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderZander.cpp
              ${XCSOAR_SRC}/Waypoint/WaypointReaderCompeGPS.cpp
              ${XCSOAR_SRC}/Waypoint/CupWriter.cpp
              ${XCSOAR_SRC}/Waypoint/Factory.cpp
              ${XCSOAR_SRC}/Units/Units.cpp
              ${XCSOAR_SRC}/Units/System.cpp
              ${XCSOAR_SRC}/Task/Serialiser.cpp
              ${XCSOAR_SRC}/Task/Deserialiser.cpp
              ${XCSOAR_SRC}/Task/SaveFile.cpp
              ${XCSOAR_SRC}/Task/LoadFile.cpp
              ${XCSOAR_SRC}/Task/TaskFile.cpp
              ${XCSOAR_SRC}/Task/TaskFileXCSoar.cpp
              ${XCSOAR_SRC}/Task/TaskFileIGC.cpp
              ${XCSOAR_SRC}/Task/TaskFileSeeYou.cpp
              ${XCSOAR_SRC}/Task/DefaultTask.cpp
              ${XCSOAR_SRC}/Task/MapTaskManager.cpp
              ${XCSOAR_SRC}/Task/ProtectedTaskManager.cpp
              ${XCSOAR_SRC}/Task/FileProtectedTaskManager.cpp
              ${XCSOAR_SRC}/Task/RoutePlannerGlue.cpp
              ${XCSOAR_SRC}/Task/ProtectedRoutePlanner.cpp
              ${XCSOAR_SRC}/Task/TaskStore.cpp
              ${XCSOAR_SRC}/Task/TypeStrings.cpp
              ${XCSOAR_SRC}/Task/ValidationErrorStrings.cpp
              ${XCSOAR_SRC}/Operation/Operation.cpp
              ${XCSOAR_SRC}/RadioFrequency.cpp
              ${XCSOAR_SRC}/Units/Descriptor.cpp
              ${XCSOAR_SRC}/XML/Node.cpp
              ${XCSOAR_SRC}/XML/Parser.cpp
              ${XCSOAR_SRC}/XML/Writer.cpp
              ${XCSOAR_SRC}/XML/DataNode.cpp
              ${XCSOAR_SRC}/XML/DataNodeXML.cpp
              ${XCSOAR_SRC}/Formatter/Units.cpp
              ${XCSOAR_SRC}/Formatter/UserUnits.cpp
              ${XCSOAR_SRC}/Formatter/HexColor.cpp
              ${XCSOAR_SRC}/Formatter/GlideRatioFormatter.cpp
              ${XCSOAR_SRC}/Formatter/GeoPointFormatter.cpp
              ${XCSOAR_SRC}/Formatter/ByteSizeFormatter.cpp
              ${XCSOAR_SRC}/Formatter/UserGeoPointFormatter.cpp
              ${XCSOAR_SRC}/Formatter/TimeFormatter.cpp
              ${XCSOAR_SRC}/Formatter/LocalTimeFormatter.cpp
              ${XCSOAR_SRC}/Formatter/IGCFilenameFormatter.cpp
              ${XCSOAR_SRC}/Formatter/AirspaceFormatter.cpp
              ${XCSOAR_SRC}/Formatter/AirspaceUserUnitsFormatter.cpp
              ${XCSOAR_SRC}/Engine/Util/Gradient.cpp
              ${XCSOAR_SRC}/Renderer/OZPreviewRenderer.cpp
              ${XCSOAR_SRC}/UIState.cpp
              ${XCSOAR_SRC}/UISettings.cpp
              ${XCSOAR_SRC}/SystemSettings.cpp
              ${XCSOAR_SRC}/Computer/Settings.cpp
              ${XCSOAR_SRC}/PageState.cpp
              ${XCSOAR_SRC}/DisplaySettings.cpp
              ${XCSOAR_SRC}/MapSettings.cpp
              ${XCSOAR_SRC}/InfoBoxes/InfoBoxSettings.cpp
              ${XCSOAR_SRC}/Gauge/VarioSettings.cpp
              ${XCSOAR_SRC}/Audio/VarioSettings.cpp
              ${XCSOAR_SRC}/PageSettings.cpp
              ${XCSOAR_SRC}/Gauge/TrafficSettings.cpp
              ${XCSOAR_SRC}/Dialogs/DialogSettings.cpp
              ${XCSOAR_SRC}/Audio/Settings.cpp
              ${XCSOAR_SRC}/Renderer/AirspaceRendererSettings.cpp
              ${XCSOAR_SRC}/Logger/Settings.cpp
              ${XCSOAR_SRC}/Airspace/AirspaceComputerSettings.cpp
              ${XCSOAR_SRC}/TeamCode/Settings.cpp
              ${XCSOAR_SRC}/Computer/Wind/Settings.cpp
              ${XCSOAR_SRC}/Device/Config.cpp
              ${XCSOAR_SRC}/Units/Settings.cpp
              )
set(MAIN_HDRS ${XCSOAR_SRC}/ActionInterface.hpp
              ${XCSOAR_SRC}/ApplyExternalSettings.hpp
              ${XCSOAR_SRC}/ApplyVegaSwitches.hpp
              ${XCSOAR_SRC}/Asset.hpp
              ${XCSOAR_SRC}/BallastDumpManager.hpp
              ${XCSOAR_SRC}/BatteryTimer.hpp
              ${XCSOAR_SRC}/CalculationThread.hpp
              ${XCSOAR_SRC}/CommandLine.hpp
              ${XCSOAR_SRC}/Components.hpp
              ${XCSOAR_SRC}/DataGlobals.hpp
              ${XCSOAR_SRC}/DisplayMode.hpp
              ${XCSOAR_SRC}/DisplayOrientation.hpp
              ${XCSOAR_SRC}/DisplaySettings.hpp
              ${XCSOAR_SRC}/DrawThread.hpp
              ${XCSOAR_SRC}/FlightInfo.hpp
              ${XCSOAR_SRC}/FlightStatistics.hpp
              ${XCSOAR_SRC}/FormatSettings.hpp
              ${XCSOAR_SRC}/HexDump.hpp
              ${XCSOAR_SRC}/HorizonWidget.hpp
              ${XCSOAR_SRC}/Interface.hpp
              ${XCSOAR_SRC}/LocalPath.hpp
              ${XCSOAR_SRC}/LogFile.hpp
              ${XCSOAR_SRC}/MainWindow.hpp
              ${XCSOAR_SRC}/MapSettings.hpp
              ${XCSOAR_SRC}/MergeThread.hpp
              ${XCSOAR_SRC}/Message.hpp
              ${XCSOAR_SRC}/PageActions.hpp
              ${XCSOAR_SRC}/PageSettings.hpp
              ${XCSOAR_SRC}/PageState.hpp
              ${XCSOAR_SRC}/Pan.hpp
              ${XCSOAR_SRC}/PopupMessage.hpp
              ${XCSOAR_SRC}/ProcessTimer.hpp
              ${XCSOAR_SRC}/ProgressGlue.hpp
              ${XCSOAR_SRC}/ProgressWindow.hpp
              ${XCSOAR_SRC}/Protection.hpp
              ${XCSOAR_SRC}/RadioFrequency.hpp
              ${XCSOAR_SRC}/RateLimiter.hpp
              ${XCSOAR_SRC}/ResourceId.hpp
              ${XCSOAR_SRC}/ResourceLoader.hpp
              ${XCSOAR_SRC}/Resources.hpp
              ${XCSOAR_SRC}/Simulator.hpp
              ${XCSOAR_SRC}/Startup.hpp
              ${XCSOAR_SRC}/StatusMessage.hpp
              ${XCSOAR_SRC}/SystemSettings.hpp
              ${XCSOAR_SRC}/TeamActions.hpp
              ${XCSOAR_SRC}/UIActions.hpp
              ${XCSOAR_SRC}/UIGlobals.hpp
              ${XCSOAR_SRC}/UIReceiveBlackboard.hpp
              ${XCSOAR_SRC}/UISettings.hpp
              ${XCSOAR_SRC}/UIState.hpp
              ${XCSOAR_SRC}/UtilsSettings.hpp
              ${XCSOAR_SRC}/UtilsSystem.hpp
              ${XCSOAR_SRC}/Version.hpp)
set(WAYPOINT_HDRS ${XCSOAR_SRC}/Waypoint/CupWriter.hpp
                  ${XCSOAR_SRC}/Waypoint/Factory.hpp
                  ${XCSOAR_SRC}/Waypoint/LastUsed.hpp
                  ${XCSOAR_SRC}/Waypoint/Patterns.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointDetailsReader.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointFileType.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointFilter.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointGlue.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointListBuilder.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointList.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderBase.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderCompeGPS.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderFS.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReader.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderOzi.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderSeeYou.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderWinPilot.hpp
                  ${XCSOAR_SRC}/Waypoint/WaypointReaderZander.hpp)
set(UNITS_HDRS ${XCSOAR_SRC}/Units/Descriptor.hpp
               ${XCSOAR_SRC}/Units/Group.hpp
               ${XCSOAR_SRC}/Units/Settings.hpp
               ${XCSOAR_SRC}/Units/System.hpp
               ${XCSOAR_SRC}/Units/Unit.hpp
               ${XCSOAR_SRC}/Units/UnitsGlue.hpp
               ${XCSOAR_SRC}/Units/Units.hpp
               ${XCSOAR_SRC}/Units/UnitsStore.hpp)
set(TASK_HDRS ${XCSOAR_SRC}/Task/DefaultTask.hpp
              ${XCSOAR_SRC}/Task/Deserialiser.hpp
              ${XCSOAR_SRC}/Task/LoadFile.hpp
              ${XCSOAR_SRC}/Task/MapTaskManager.hpp
              ${XCSOAR_SRC}/Task/ProtectedRoutePlanner.hpp
              ${XCSOAR_SRC}/Task/ProtectedTaskManager.hpp
              ${XCSOAR_SRC}/Task/RoutePlannerGlue.hpp
              ${XCSOAR_SRC}/Task/SaveFile.hpp
              ${XCSOAR_SRC}/Task/Serialiser.hpp
              ${XCSOAR_SRC}/Task/TaskFile.hpp
              ${XCSOAR_SRC}/Task/TaskFileIGC.hpp
              ${XCSOAR_SRC}/Task/TaskFileSeeYou.hpp
              ${XCSOAR_SRC}/Task/TaskFileXCSoar.hpp
              ${XCSOAR_SRC}/Task/TaskStore.hpp
              ${XCSOAR_SRC}/Task/TypeStrings.hpp
              ${XCSOAR_SRC}/Task/ValidationErrorStrings.hpp)
set(OPERATION_HDRS ${XCSOAR_SRC}/Operation/ConsoleOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/MessageOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/NoCancelOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/Operation.hpp
                   ${XCSOAR_SRC}/Operation/PopupOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/ProxyOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/ThreadedOperationEnvironment.hpp
                   ${XCSOAR_SRC}/Operation/VerboseOperationEnvironment.hpp)
set(XML_HDRS ${XCSOAR_SRC}/XML/DataNode.hpp
             ${XCSOAR_SRC}/XML/DataNodeXML.hpp
             ${XCSOAR_SRC}/XML/Node.hpp
             ${XCSOAR_SRC}/XML/Parser.hpp)
set(FORMATTER_HDRS ${XCSOAR_SRC}/Formatter/AirspaceFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/AngleFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/ByteSizeFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/GeoPointFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/GlideRatioFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/HexColor.hpp
                   ${XCSOAR_SRC}/Formatter/IGCFilenameFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/LocalTimeFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/TimeFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/Units.hpp
                   ${XCSOAR_SRC}/Formatter/UserGeoPointFormatter.hpp
                   ${XCSOAR_SRC}/Formatter/UserUnits.hpp)
include_directories(${XCSOAR_SRC})
add_library(XCSoarMain-static-${T} STATIC ${MAIN_SRCS})
add_library(XCSoarMain-shared-${T} SHARED ${MAIN_SRCS})
add_library(XCSoarMain-object-${T} OBJECT ${MAIN_SRCS})
target_link_libraries(XCSoarMain-static-${T} Profile-static-${T}
                                             Os-static-${T}
                                             Util-static-${T}
                                             ContestEngine-static-${T}
                                             AirspaceEngine-static-${T})
target_link_libraries(XCSoarMain-shared-${T} Profile-shared-${T}
                                             Os-shared-${T}
                                             Util-shared-${T}
                                             ContestEngine-shared-${T}
                                             AirspaceEngine-shared-${T})
set_target_properties(XCSoarMain-static-${T} 
                      PROPERTIES COMPILE_FLAGS "-fexceptions")
set_target_properties(XCSoarMain-shared-${T}
                      PROPERTIES COMPILE_FLAGS "-fexceptions")
set_target_properties(XCSoarMain-object-${T}
                      PROPERTIES COMPILE_FLAGS "-fexceptions")

add_library(XCSoar-static-${T} STATIC $<TARGET_OBJECTS:XCSoarMain-object-${T}>
                                      $<TARGET_OBJECTS:Util-object-${T}>
                                      $<TARGET_OBJECTS:Geo-object-${T}>
                                      $<TARGET_OBJECTS:Math-object-${T}>
                                      $<TARGET_OBJECTS:Io-object-${T}>
                                      $<TARGET_OBJECTS:WaypointEngine-object-${T}>
                                      $<TARGET_OBJECTS:RouteEngine-object-${T}>
                                      $<TARGET_OBJECTS:GlideEngine-object-${T}>
                                      $<TARGET_OBJECTS:ContestEngine-object-${T}>
                                      $<TARGET_OBJECTS:TaskEngine-object-${T}>
                                      $<TARGET_OBJECTS:AirspaceEngine-object-${T}>
                                      $<TARGET_OBJECTS:Driver-object-${T}>
                                      $<TARGET_OBJECTS:Shape-object-${T}>
                                      $<TARGET_OBJECTS:Os-object-${T}>
                                      $<TARGET_OBJECTS:Thread-object-${T}>
                                      $<TARGET_OBJECTS:Terrain-object-${T}>
                                      $<TARGET_OBJECTS:Profile-object-${T}>
                                      $<TARGET_OBJECTS:Time-object-${T}>
                                      $<TARGET_OBJECTS:Screen-object-${T}>
                                      $<TARGET_OBJECTS:Jasper-object-${T}>
                                      $<TARGET_OBJECTS:Zzip-object-${T}>)
add_library(XCSoar-shared-${T} SHARED $<TARGET_OBJECTS:XCSoarMain-object-${T}>
                                      $<TARGET_OBJECTS:Util-object-${T}>
                                      $<TARGET_OBJECTS:Geo-object-${T}>
                                      $<TARGET_OBJECTS:Math-object-${T}>
                                      $<TARGET_OBJECTS:Io-object-${T}>
                                      $<TARGET_OBJECTS:WaypointEngine-object-${T}>
                                      $<TARGET_OBJECTS:RouteEngine-object-${T}>
                                      $<TARGET_OBJECTS:GlideEngine-object-${T}>
                                      $<TARGET_OBJECTS:ContestEngine-object-${T}>
                                      $<TARGET_OBJECTS:TaskEngine-object-${T}>
                                      $<TARGET_OBJECTS:AirspaceEngine-object-${T}>
                                      $<TARGET_OBJECTS:Driver-object-${T}>
                                      $<TARGET_OBJECTS:Shape-object-${T}>
                                      $<TARGET_OBJECTS:Os-object-${T}>
                                      $<TARGET_OBJECTS:Thread-object-${T}>
                                      $<TARGET_OBJECTS:Terrain-object-${T}>
                                      $<TARGET_OBJECTS:Profile-object-${T}>
                                      $<TARGET_OBJECTS:Time-object-${T}>
                                      $<TARGET_OBJECTS:Screen-object-${T}>
                                      $<TARGET_OBJECTS:Jasper-object-${T}>
                                      $<TARGET_OBJECTS:Zzip-object-${T}>)
target_link_libraries(XCSoar-static-${T} pthread z)
target_link_libraries(XCSoar-shared-${T} pthread z)
install(FILES ${MAIN_HDRS}      DESTINATION "include")
install(FILES ${WAYPOINT_HDRS}  DESTINATION "include/Waypoint")
install(FILES ${UNITS_HDRS}     DESTINATION "include/Units")
install(FILES ${TASK_HDRS}      DESTINATION "include/Task")
install(FILES ${OPERATION_HDRS} DESTINATION "include/Operation")
install(FILES ${XML_HDRS}       DESTINATION "include/XML")
install(FILES ${FORMATTER_HDRS} DESTINATION "include/Formatter")

add_custom_target(xcsoar-${T}
                  DEPENDS AirspaceEngine-static-${T}
                          AirspaceEngine-shared-${T}
                          ContestEngine-static-${T}
                          ContestEngine-shared-${T}
                          Driver-static-${T}
                          Driver-shared-${T}
                          Geo-static-${T}
                          Geo-shared-${T}
                          Terrain-static-${T}
                          Terrain-shared-${T}
                          Jasper-static-${T}
                          Jasper-shared-${T}
                          Zzip-static-${T}
                          Zzip-shared-${T}
                          Profile-static-${T}
                          Profile-shared-${T}
                          GlideEngine-static-${T}
                          GlideEngine-shared-${T}
                          Io-static-${T}
                          Io-shared-${T}
                          Math-static-${T}
                          Math-shared-${T}
                          RouteEngine-static-${T}
                          RouteEngine-shared-${T}
                          Shape-static-${T}
                          Shape-shared-${T}
                          TaskEngine-static-${T}
                          TaskEngine-shared-${T}
                          Util-static-${T}
                          Util-shared-${T}
                          Time-static-${T}
                          Time-shared-${T}
                          Screen-static-${T}
                          Screen-shared-${T}
                          WaypointEngine-static-${T}
                          WaypointEngine-shared-${T}
                          Os-static-${T}
                          Os-shared-${T}
                          Thread-static-${T}
                          Thread-shared-${T}
                          XCSoarMain-static-${T}
                          XCSoarMain-shared-${T}
                          XCSoar-static-${T}
                          XCSoar-shared-${T})
add_custom_target(${T} DEPENDS xcsoar-${T})

set(CMAKE_INSTALL_PREFIX "${XCSOAR}/out" CACHE PATH "..." FORCE)
set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT TRUE)
install(TARGETS Util-shared-${T}
                Geo-shared-${T}
                AirspaceEngine-shared-${T}
                ContestEngine-shared-${T}
                Driver-shared-${T}
                GlideEngine-shared-${T}
                Io-shared-${T}
                Jasper-shared-${T}
                Math-shared-${T}
                Os-shared-${T}
                Profile-shared-${T}
                RouteEngine-shared-${T}
                Screen-shared-${T}
                Shape-shared-${T}
                TaskEngine-shared-${T}
                Terrain-shared-${T}
                Thread-shared-${T}
                Time-shared-${T}
                WaypointEngine-shared-${T}
                XCSoar-shared-${T}
                XCSoarMain-shared-${T}
                Zzip-shared-${T}
        EXPORT XCSoar-lib
        DESTINATION lib)
install(EXPORT XCSoar-lib DESTINATION lib)
