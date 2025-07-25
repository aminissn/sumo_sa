set(libsumostatic_STAT_SRCS
    Edge.cpp
    Edge.h
    Helper.cpp
    Helper.h
    InductionLoop.cpp
    InductionLoop.h
    Junction.cpp
    Junction.h
    Lane.cpp
    Lane.h
    LaneArea.cpp
    LaneArea.h
    MultiEntryExit.h
    MultiEntryExit.cpp
    Person.cpp
    Person.h
    POI.cpp
    POI.h
    Polygon.h
    Polygon.cpp
    Route.h
    Route.cpp
    Calibrator.h
    Calibrator.cpp
    BusStop.h
    BusStop.cpp
    ParkingArea.h
    ParkingArea.cpp
    ChargingStation.h
    ChargingStation.cpp
    RouteProbe.h
    RouteProbe.cpp
    Rerouter.h
    Rerouter.cpp
    VariableSpeedSign.h
    VariableSpeedSign.cpp
    MeanData.h
    MeanData.cpp
    OverheadWire.h
    OverheadWire.cpp
    Simulation.cpp
    Simulation.h
    Subscription.h
    StorageHelper.h
    TraCIConstants.h
    TraCIDefs.h
    TrafficLight.cpp
    TrafficLight.h
    Vehicle.cpp
    Vehicle.h
    VehicleType.cpp
    VehicleType.h
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
add_library(libsumostatic STATIC ${libsumostatic_STAT_SRCS})
add_dependencies(libsumostatic generate-version-h install_dll)
set_target_properties(libsumostatic PROPERTIES FOLDER "libsumo" PREFIX ""
    OUTPUT_NAME libsumostatic${BINARY_SUFFIX}
    OUTPUT_NAME_DEBUG libsumostatic${BINARY_SUFFIX}D)

if (FOX_FOUND)
    add_library(libsumoguistatic STATIC ${libsumostatic_STAT_SRCS} GUI.cpp GUI.h)
    add_dependencies(libsumoguistatic generate-version-h install_dll)
    set_target_properties(libsumoguistatic PROPERTIES FOLDER "libsumo" PREFIX ""
        OUTPUT_NAME libsumoguistatic${BINARY_SUFFIX}
        OUTPUT_NAME_DEBUG libsumoguistatic${BINARY_SUFFIX}D)
    target_compile_definitions(libsumoguistatic PUBLIC HAVE_LIBSUMOGUI)
    add_library(libsumocpp SHARED ${libsumostatic_STAT_SRCS} GUI.cpp GUI.h)
    target_compile_definitions(libsumocpp PUBLIC HAVE_LIBSUMOGUI)
    set(libsumo_link_library libsumoguistatic)
    set(sumo_link_libraries ${sumoguilibs})
else ()
    add_library(libsumocpp SHARED ${libsumostatic_STAT_SRCS})
    set(libsumo_link_library libsumostatic)
    set(sumo_link_libraries ${sumolibs})
endif()
add_dependencies(libsumocpp generate-version-h install_dll)

set_target_properties(libsumocpp PROPERTIES FOLDER "libsumo" PREFIX ""
    OUTPUT_NAME libsumocpp${BINARY_SUFFIX}
    OUTPUT_NAME_DEBUG libsumocpp${BINARY_SUFFIX}D)
target_include_directories(libsumocpp PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>)
install(TARGETS libsumocpp EXPORT SUMOConfig
    DESTINATION bin
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
export(TARGETS libsumocpp NAMESPACE SUMO:: FILE "${CMAKE_BINARY_DIR}/SUMOConfig.cmake")

if ((GNU_COMPILER OR CLANG_COMPILER) AND NOT APPLE)
    target_link_libraries(libsumocpp PRIVATE -Wl,--whole-archive ${sumolibs} -Wl,--no-whole-archive)
else ()
    target_link_libraries(libsumocpp PRIVATE ${sumolibs})
endif()
target_link_libraries(libsumocpp PRIVATE ${sumo_link_libraries})

if (SWIG_FOUND)
    include(${SWIG_USE_FILE})
    set_source_files_properties(libsumo.i PROPERTIES CPLUSPLUS ON)
    file(GLOB LIBSUMO_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    if (ENABLE_JAVA_BINDINGS)
        find_package(Java)
        find_package(JNI)
        if(JAVA_FOUND AND JNI_FOUND)
            include(UseJava)
            include_directories(${JNI_INCLUDE_DIRS})
            set(CMAKE_SWIG_FLAGS -package org.eclipse.sumo.libsumo)
            if (FOX_FOUND)
                set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} -DHAVE_LIBSUMOGUI)
            endif()
            set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java)
            set(SWIG_MODULE_libsumojni_EXTRA_DEPS ${LIBSUMO_HEADERS} libsumo_typemap.i)
            if (${CMAKE_VERSION} VERSION_GREATER 3.8.0)
                SWIG_ADD_LIBRARY(libsumojni LANGUAGE java SOURCES libsumo.i)
            else()
                SWIG_ADD_MODULE(libsumojni java libsumo.i)
            endif()
            if ((GNU_COMPILER OR CLANG_COMPILER) AND NOT APPLE)
                set_source_files_properties(${swig_generated_file_fullname} PROPERTIES COMPILE_FLAGS "-Wno-strict-aliasing")
                swig_link_libraries(libsumojni -Wl,--whole-archive ${libsumo_link_library} ${sumolibs} -Wl,--no-whole-archive)
            else()
                swig_link_libraries(libsumojni ${libsumo_link_library} ${sumolibs})
            endif()
            swig_link_libraries(libsumojni ${sumo_link_libraries})
            if (MVN_EXECUTABLE)
                ADD_CUSTOM_COMMAND(TARGET libsumojni POST_BUILD
                    COMMAND ${PYTHON_EXECUTABLE} ${SUMO_TOOLS_DIR}/traci/rebuildConstants.py -o java/Constants.java -j org.eclipse.sumo.libsumo.Constants
                    COMMAND ${CMAKE_COMMAND} -E make_directory src/main/java/org/eclipse/sumo/libsumo
                    COMMAND ${CMAKE_COMMAND} -E copy_directory java src/main/java/org/eclipse/sumo/libsumo
                    COMMAND ${PYTHON_EXECUTABLE} ${SUMO_TOOLS_DIR}/build_config/pom.py libsumo
                    COMMAND ${MVN_COMMAND_PREFIX} ${MVN_EXECUTABLE} ${MVN_REPO} ${MVN_OPTS} --batch-mode package source:jar
                )
            else()
                message(WARNING "Could not build libsumo jar, maven was not found.")
            endif()
            set_target_properties(libsumojni PROPERTIES FOLDER "libsumo")
            install(TARGETS libsumojni DESTINATION bin LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
        else()
            message(WARNING "ENABLE_JAVA_BINDINGS is set but Java or JNI was not found.")
        endif()
    endif()

    if (ENABLE_CS_BINDINGS)
        set(CS_NAMESPACE Eclipse.Sumo.Libsumo)
        set(CMAKE_SWIG_FLAGS -namespace ${CS_NAMESPACE})
        if (FOX_FOUND)
            set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} -DHAVE_LIBSUMOGUI)
        endif()
        set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/${CS_NAMESPACE})
        set(SWIG_MODULE_libsumocs_EXTRA_DEPS ${LIBSUMO_HEADERS} libsumo_typemap.i)
        if (${CMAKE_VERSION} VERSION_GREATER 3.8.0)
            SWIG_ADD_LIBRARY(libsumocs LANGUAGE CSharp SOURCES libsumo.i)
        else()
            SWIG_ADD_MODULE(libsumocs CSharp libsumo.i)
        endif()
        if ((GNU_COMPILER OR CLANG_COMPILER) AND NOT APPLE)
            set_source_files_properties(${swig_generated_file_fullname} PROPERTIES COMPILE_FLAGS "-Wno-strict-aliasing")
            swig_link_libraries(libsumocs -Wl,--whole-archive ${libsumo_link_library} ${sumolibs} -Wl,--no-whole-archive)
        else()
            swig_link_libraries(libsumocs ${libsumo_link_library} ${sumolibs})
        endif()
        swig_link_libraries(libsumocs ${sumo_link_libraries})
        ADD_CUSTOM_COMMAND(TARGET libsumocs POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E tar "cf" "${CMAKE_SOURCE_DIR}/bin/libsumocs-sources.zip" --format=zip "${CS_NAMESPACE}"
        )
        set_target_properties(libsumocs PROPERTIES FOLDER "libsumo" PREFIX "")
        install(TARGETS libsumocs DESTINATION bin LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    endif()

    if (ENABLE_PYTHON_BINDINGS AND PYTHON_INCLUDE_DIRS)
        include_directories(${PYTHON_INCLUDE_DIRS})
        set(CMAKE_SWIG_FLAGS -keyword) # is necessary for using keyword args
        # other options which should be evaluated: -builtin, -modern, -extranative
        if (FOX_FOUND)
            set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} -DHAVE_LIBSUMOGUI)
        endif()
        set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
        set(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/tools/libsumo)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/libsumo)
#            set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/tools/libsumo)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/tools/libsumo)
        set(SWIG_MODULE_libsumo_EXTRA_DEPS ${LIBSUMO_HEADERS} libsumo_typemap.i)
        if (${CMAKE_VERSION} VERSION_GREATER 3.8.0)
            if (CLANG_COMPILER)
                set_property(SOURCE libsumo.i PROPERTY GENERATED_COMPILE_OPTIONS "-Wno-deprecated-declarations")
            endif()
            SWIG_ADD_LIBRARY(libsumo LANGUAGE python SOURCES libsumo.i)
        else()
            SWIG_ADD_MODULE(libsumo python libsumo.i)
        endif()
        add_dependencies(${SWIG_MODULE_libsumo_REAL_NAME} generate-version-h install_dll)
        if ((GNU_COMPILER OR CLANG_COMPILER) AND NOT APPLE)
            swig_link_libraries(libsumo -Wl,--whole-archive ${libsumo_link_library} ${sumolibs} -Wl,--no-whole-archive)
        else()
            set_target_properties(${SWIG_MODULE_libsumo_REAL_NAME} PROPERTIES FOLDER "libsumo")
            if (SKBUILD OR ${CMAKE_VERSION} VERSION_LESS 3.14.0)
                swig_link_libraries(libsumo ${libsumo_link_library} ${sumolibs} ${PYTHON_LIBRARIES})
            else()
                # using target fixes segfault in macos, swig_link_libraries is also deprecated
                target_link_libraries(libsumo ${libsumo_link_library} ${sumolibs} Python::Module)
            endif()
        endif()
        if (WIN32)
            swig_link_libraries(libsumo ${PYTHON_LIBRARIES})
        endif()
        swig_link_libraries(libsumo ${sumo_link_libraries})
        if (NOT ${PYTHON_SETUPTOOLS_MISSING})
            if (${PYTHON_BUILD_MISSING})
                install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${SUMO_TOOLS_DIR}/build_config/setup-libsumo.py clean --all install --root=\$ENV{DESTDIR}/ --prefix=${CMAKE_INSTALL_PREFIX} --optimize=1)"
                        COMPONENT pylibsumo)
            else ()
                install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ./build_config/version.py ./build_config/setup-libsumo.py ./setup.py COMMAND ${PYTHON_EXECUTABLE} -m pip install --root=\$ENV{DESTDIR}/ --prefix=${CMAKE_INSTALL_PREFIX} -f ${CMAKE_BINARY_DIR} . WORKING_DIRECTORY ${SUMO_TOOLS_DIR})"
                        COMPONENT pylibsumo)
            endif()
        endif()
    endif()

endif()
