set(XCSOAR ${XCSoar-the-library_SOURCE_DIR})
set(XCSOAR_SRC ${XCSOAR}/src)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)

include_directories(${XCSOAR_SRC})
add_definitions(-DZLIB_CONST) # Make some of zlib const.
add_definitions(-DUSE_WGS84 -DRADIANS -DEYE_CANDY)

set(UTIL_DIR ${XCSOAR_SRC}/Util)
set(UTIL_SRCS ${UTIL_DIR}/CRC.cpp
             ${UTIL_DIR}/EscapeBackslash.cpp
             ${UTIL_DIR}/StringUtil.cpp
             ${UTIL_DIR}/ConvertString.cpp
             ${UTIL_DIR}/ExtractParameters.cpp
             ${UTIL_DIR}/UTF8.cpp
             ${UTIL_DIR}/tstring.cpp)
include_directories(${XCSOAR_SRC} ${XCSOAR_SRC}/unix ${UTIL_DIR})
add_library(Util-static-${T} STATIC ${UTIL_SRCS})
add_library(Util-shared-${T} SHARED ${UTIL_SRCS})

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
include_directories(${XCSOAR_SRC} ${GEO_DIR})
add_library(Geo-static-${T} STATIC ${GEO_SRCS})
add_library(Geo-shared-${T} SHARED ${GEO_SRCS})

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
include_directories(${XCSOAR_SRC} ${MATH_DIR} ${OUTPUT_INCLUDE})
add_library(Math-static-${T} STATIC ${MATH_SRCS})
add_library(Math-shared-${T} SHARED ${MATH_SRCS})

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
            ${IO_DIR}/TextWriter.cpp)
include_directories(${XCSOAR_SRC} ${IO_DIR})
add_library(Io-static-${T} STATIC ${IO_SRCS})
add_library(Io-shared-${T} SHARED ${IO_SRCS})

set(WAYPOINTENGINE_DIR ${XCSOAR_SRC}/Engine/Waypoint)
set(WAYPOINTENGINE_SRCS ${WAYPOINTENGINE_DIR}/WaypointVisitor.cpp
                        ${WAYPOINTENGINE_DIR}/Waypoints.cpp
                        ${WAYPOINTENGINE_DIR}/Waypoint.cpp)
include_directories(${XCSOAR_SRC} ${WAYPOINTENGINE_DIR})
add_library(WaypointEngine-static-${T} STATIC ${WAYPOINTENGINE_SRCS})
add_library(WaypointEngine-shared-${T} SHARED ${WAYPOINTENGINE_SRCS})
target_link_libraries(WaypointEngine-static-${T} Geo-static-${T} Math-static-${T})
target_link_libraries(WaypointEngine-shared-${T} Geo-shared-${T} Math-shared-${T})

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
include_directories(${XCSOAR_SRC} ${ROUTEENGINE_DIR} ${XCSOAR_SRC}/Engine)
add_library(RouteEngine-static-${T} STATIC ${ROUTEENGINE_SRCS})
add_library(RouteEngine-shared-${T} SHARED ${ROUTEENGINE_SRCS})

set(GLIDEENGINE_DIR ${XCSOAR_SRC}/Engine/GlideSolvers)
set(GLIDEENGINE_SRCS ${GLIDEENGINE_DIR}/GlideSettings.cpp
                     ${GLIDEENGINE_DIR}/GlideState.cpp
                     ${GLIDEENGINE_DIR}/GlueGlideState.cpp
                     ${GLIDEENGINE_DIR}/GlidePolar.cpp
                     ${GLIDEENGINE_DIR}/PolarCoefficients.cpp
                     ${GLIDEENGINE_DIR}/GlideResult.cpp
                     ${GLIDEENGINE_DIR}/MacCready.cpp)
include_directories(${XCSOAR_SRC} ${GLIDEENGINE_DIR})
add_library(GlideEngine-static-${T} STATIC ${GLIDEENGINE_SRCS})
add_library(GlideEngine-shared-${T} SHARED ${GLIDEENGINE_SRCS})

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
include_directories(${XCSOAR_SRC} ${CONTESTENGINE_DIR})
add_library(ContestEngine-static-${T} STATIC ${CONTESTENGINE_SRCS})
add_library(ContestEngine-shared-${T} SHARED ${CONTESTENGINE_SRCS})

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
include_directories(${XCSOAR_SRC} ${TASKENGINE_DIR})
add_library(TaskEngine-static-${T} STATIC ${TASKENGINE_SRCS})
add_library(TaskEngine-shared-${T} SHARED ${TASKENGINE_SRCS})
# TaskEngine should not depend on XCSoarMain but is made necessary because of
# the sematics of the original XCSoar build system.
target_link_libraries(TaskEngine-static-${T} GlideEngine-static-${T} 
                                             RouteEngine-static-${T}
                                             XCSoarMain-static-${T})
target_link_libraries(TaskEngine-shared-${T} GlideEngine-shared-${T} 
                                             RouteEngine-shared-${T}
                                             XCSoarMain-shared-${T})

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
include_directories(${XCSOAR_SRC} ${AIRSPACEENGINE_DIR})
add_library(AirspaceEngine-static-${T} STATIC ${AIRSPACEENGINE_SRCS})
add_library(AirspaceEngine-shared-${T} SHARED ${AIRSPACEENGINE_SRCS})

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
set(CAI302_SRCS ${DRIVER_DIR}/CAI302/Protocol.cpp
                ${DRIVER_DIR}/CAI302/PocketNav.cpp
                ${DRIVER_DIR}/CAI302/Mode.cpp
                ${DRIVER_DIR}/CAI302/Parser.cpp
                ${DRIVER_DIR}/CAI302/Settings.cpp
                ${DRIVER_DIR}/CAI302/Declare.cpp
                ${DRIVER_DIR}/CAI302/Logger.cpp
                ${DRIVER_DIR}/CAI302/Manage.cpp
                ${DRIVER_DIR}/CAI302/Register.cpp)
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
set(FLYTEC_SRCS ${DRIVER_DIR}/Flytec/Register.cpp
                ${DRIVER_DIR}/Flytec/Parser.cpp
                ${DRIVER_DIR}/Flytec/Logger.cpp)
set(VEGA_SRCS ${DRIVER_DIR}/Vega/Misc.cpp
              ${DRIVER_DIR}/Vega/Parser.cpp
              ${DRIVER_DIR}/Vega/Settings.cpp
              ${DRIVER_DIR}/Vega/Volatile.cpp
              ${DRIVER_DIR}/Vega/Register.cpp)
set(BLUEFLY_SRCS ${DRIVER_DIR}/BlueFly/Misc.cpp
                 ${DRIVER_DIR}/BlueFly/Parser.cpp
                 ${DRIVER_DIR}/BlueFly/Settings.cpp
                 ${DRIVER_DIR}/BlueFly/Register.cpp)
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
               ${DRIVER_DIR}/GTAltimeter.cpp
               ${DRIVER_DIR}/NmeaOut.cpp
               ${DRIVER_DIR}/OpenVario.cpp
               ${DRIVER_DIR}/PosiGraph.cpp
               ${DRIVER_DIR}/XCOM760.cpp
               ${DRIVER_DIR}/ILEC.cpp
               ${DRIVER_DIR}/Westerboer.cpp
               ${DRIVER_DIR}/Zander.cpp
               ${DRIVER_DIR}/Vaulter.cpp
               ${DRIVER_DIR}/ATR833.cpp)
set(DRIVER_SRCS ${VOLKSLOGGER_SRCS}
                ${CAI302_SRCS}
                ${IMI_SRCS}
                ${LX_SRCS}
                ${FLARM_SRCS}
                ${FLYTEC_SRCS}
                ${VEGA_SRCS}
                ${BLUEFLY_SRCS}
                ${OTHER_SRCS})
include_directories(${XCSOAR_SRC} ${DRIVER_DIR})
#set(OUTPUT_NAME Driver-${T})
add_library(Driver-static-${T} STATIC ${DRIVER_SRCS})
add_library(Driver-shared-${T} SHARED ${DRIVER_SRCS})

set(SHAPE_DIR ${XCSOAR_SRC}/Topography/shapelib)
set(SHAPE_SRCS ${SHAPE_DIR}/mapalloc.c
               ${SHAPE_DIR}/mapstring.c
               ${SHAPE_DIR}/mapbits.c
               ${SHAPE_DIR}/mapprimitive.c
               ${SHAPE_DIR}/mapsearch.c
               ${SHAPE_DIR}/mapshape.c
               ${SHAPE_DIR}/maptree.c
               ${SHAPE_DIR}/mapxbase.c)
include_directories(${XCSOAR_SRC} ${SHAPE_DIR})
add_library(Shape-static-${T} STATIC ${SHAPE_SRCS})
add_library(Shape-shared-${T} SHARED ${SHAPE_SRCS})

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
if(HAVE_POSIX)
  set(OS_SRCS ${OS_SRCS} ${OS_DIR}/EventPipe.cpp)
endif(HAVE_POSIX)
add_library(Os-static-${T} STATIC ${OS_SRCS})
add_library(Os-shared-${T} SHARED ${OS_SRCS})

set(THREAD_DIR ${XCSOAR_SRC}/Thread)
set(THREAD_SRCS	${THREAD_DIR}/Thread.cpp
                ${THREAD_DIR}/SuspensibleThread.cpp
                ${THREAD_DIR}/RecursivelySuspensibleThread.cpp
                ${THREAD_DIR}/WorkerThread.cpp
                ${THREAD_DIR}/StandbyThread.cpp
                ${THREAD_DIR}/Debug.cpp)
add_library(Thread-static-${T} STATIC ${THREAD_SRCS})
add_library(Thread-shared-${T} SHARED ${THREAD_SRCS})
target_link_libraries(Thread-static-${T} pthread)
target_link_libraries(Thread-shared-${T} pthread)

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
add_library(Terrain-static-${T} STATIC ${TERRAIN_SRCS})
add_library(Terrain-shared-${T} SHARED ${TERRAIN_SRCS})
target_link_libraries(Terrain-static-${T} Jasper-static-${T} pthread)
target_link_libraries(Terrain-shared-${T} Jasper-shared-${T} pthread)

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
add_library(Profile-static-${T} STATIC ${PROFILE_SRCS})
add_library(Profile-shared-${T} SHARED ${PROFILE_SRCS})
target_link_libraries(Profile-static-${T} Io-static-${T})
target_link_libraries(Profile-shared-${T} Io-shared-${T})

set(TIME_DIR ${XCSOAR_SRC}/Time)
set(TIME_SRCS ${TIME_DIR}/DeltaTime.cpp
              ${TIME_DIR}/WrapClock.cpp
              ${TIME_DIR}/LocalTime.cpp
              ${TIME_DIR}/BrokenTime.cpp
              ${TIME_DIR}/BrokenDate.cpp
              ${TIME_DIR}/BrokenDateTime.cpp)
add_library(Time-static-${T} STATIC ${TIME_SRCS})
add_library(Time-shared-${T} SHARED ${TIME_SRCS})

set(SCREEN_DIR ${XCSOAR_SRC}/Screen)
set(SCREEN_SRCS ${SCREEN_DIR}/Debug.cpp
#                ${SCREEN_DIR}/Custom/Cache.cpp
#                ${SCREEN_DIR}/FreeType/Font.cpp
#                ${SCREEN_DIR}/FreeType/Init.cpp
   )
add_library(Screen-static-${T} STATIC ${SCREEN_SRCS})
add_library(Screen-shared-${T} SHARED ${SCREEN_SRCS})
target_link_libraries(Screen-static-${T} Util-static-${T} pthread freetype)
target_link_libraries(Screen-shared-${T} Util-shared-${T} pthread freetype)
target_include_directories(Screen-static-${T} SYSTEM PUBLIC
                           /usr/include/freetype2)  # For Font.cpp
target_include_directories(Screen-shared-${T} SYSTEM PUBLIC
                           /usr/include/freetype2)

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
                ${JASPER_DIR}/jpc/jpc_qmfb.c
                ${JASPER_DIR}/jpc/jpc_t1dec.c
                ${JASPER_DIR}/jpc/jpc_t1cod.c
                ${JASPER_DIR}/jpc/jpc_t2dec.c
                ${JASPER_DIR}/jpc/jpc_tagtree.c)
add_library(Jasper-static-${T} STATIC ${JASPER_SRCS})
add_library(Jasper-shared-${T} SHARED ${JASPER_SRCS})
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
target_link_libraries(Jasper-static-${T} Zzip-static-${T})
target_link_libraries(Jasper-shared-${T} Zzip-shared-${T})

set(ZZIP_DIR ${XCSOAR_SRC}/zzip)
set(ZZIP_SRCS ${ZZIP_DIR}/fetch.c
              ${ZZIP_DIR}/file.c
              ${ZZIP_DIR}/plugin.c
              ${ZZIP_DIR}/zip.c
              ${ZZIP_DIR}/stat.c)
add_library(Zzip-static-${T} STATIC ${ZZIP_SRCS})
add_library(Zzip-shared-${T} SHARED ${ZZIP_SRCS})
target_link_libraries(Zzip-static-${T} z)
target_link_libraries(Zzip-shared-${T} z)

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
include_directories(${XCSOAR_SRC})
add_library(XCSoarMain-static-${T} STATIC ${MAIN_SRCS})
add_library(XCSoarMain-shared-${T} SHARED ${MAIN_SRCS})
target_link_libraries(XCSoarMain-static-${T} Profile-static-${T}
                                             Os-static-${T}
                                             Util-static-${T}
                                             ContestEngine-static-${T}
                                             AirspaceEngine-static-${T})
target_link_libraries(XCSoarMain-static-${T} Profile-shared-${T}
                                             Os-shared-${T}
                                             Util-shared-${T}
                                             ContestEngine-shared-${T}
                                             AirspaceEngine-shared-${T})
set_target_properties(XCSoarMain-static-${T} 
                      PROPERTIES COMPILE_FLAGS "-fexceptions")
set_target_properties(XCSoarMain-shared-${T}
                      PROPERTIES COMPILE_FLAGS "-fexceptions")

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
                          XCSoarMain-shared-${T})
add_custom_target(${T} DEPENDS xcsoar-${T})
