cmake_minimum_required(VERSION 3.20)

project(magazine_capacity VERSION 0.1.0 LANGUAGES CXX)

option(MAGAZINE_BUILD_GUI "Build the Qt/OCCT packing viewer" OFF)
set(MAGAZINE_OCCT_ROOT "" CACHE PATH "Complete OCCT SDK root")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

include(CTest)

add_library(packing_core
    src/packing/LatticePacking.cpp
    src/packing/QuasiStaticSettler.cpp
    src/packing/Validation.cpp
)
target_include_directories(packing_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(packing-cli apps/packing-cli/main.cpp)
target_link_libraries(packing-cli PRIVATE packing_core)

if(BUILD_TESTING)
    add_executable(packing-core-tests tests/PackingCoreTests.cpp)
    target_link_libraries(packing-core-tests PRIVATE packing_core)
    add_test(NAME packing-core-tests COMMAND packing-core-tests)
endif()

if(MAGAZINE_BUILD_GUI)
    if(NOT WIN32)
        message(FATAL_ERROR "The first OCCT viewport implementation currently targets Windows")
    endif()

    find_package(Qt5 5.15.2 EXACT REQUIRED COMPONENTS Widgets Concurrent)

    set(MAGAZINE_OCCT_LIBRARIES
        TKernel TKMath
        TKG2d TKG3d TKGeomBase TKBRep
        TKGeomAlgo TKTopAlgo TKPrim TKService TKV3d TKOpenGl
        TKDE TKDESTEP TKXSBase TKMesh
    )

    # OCCT toolkits load additional DLLs transitively at runtime.  Keep the
    # deployment closure explicit so a freshly built GUI can be launched by
    # double-clicking it without depending on the developer machine's PATH.
    set(MAGAZINE_OCCT_RUNTIME_LIBRARIES
        ${MAGAZINE_OCCT_LIBRARIES}
        TKBO TKCAF TKCDF TKHLR TKLCAF TKShHealing TKVCAF TKXCAF
    )

    if(MAGAZINE_OCCT_ROOT)
        if(EXISTS "${MAGAZINE_OCCT_ROOT}/include/opencascade/TopoDS_Shape.hxx")
            set(MAGAZINE_OCCT_INCLUDE_DIR "${MAGAZINE_OCCT_ROOT}/include/opencascade")
            set(MAGAZINE_OCCT_LIBRARY_DIR "${MAGAZINE_OCCT_ROOT}/lib")
            set(MAGAZINE_OCCT_RUNTIME_DIR "${MAGAZINE_OCCT_ROOT}/bin")
        elseif(EXISTS "${MAGAZINE_OCCT_ROOT}/inc/TopoDS_Shape.hxx")
            set(MAGAZINE_OCCT_INCLUDE_DIR "${MAGAZINE_OCCT_ROOT}/inc")
            set(MAGAZINE_OCCT_LIBRARY_DIR "${MAGAZINE_OCCT_ROOT}/win64/vc14/lib")
            set(MAGAZINE_OCCT_RUNTIME_DIR "${MAGAZINE_OCCT_ROOT}/win64/vc14/bin")
        endif()
    else()
        find_package(OpenCASCADE REQUIRED)
        set(MAGAZINE_OCCT_INCLUDE_DIR "${OpenCASCADE_INCLUDE_DIR}")
        set(MAGAZINE_OCCT_LIBRARY_DIR "${OpenCASCADE_LIBRARY_DIR}")
        set(MAGAZINE_OCCT_RUNTIME_DIR "${OpenCASCADE_LIBRARY_DIR}/../bin")
    endif()

    if(NOT EXISTS "${MAGAZINE_OCCT_INCLUDE_DIR}/TopoDS_Shape.hxx" OR
       NOT EXISTS "${MAGAZINE_OCCT_LIBRARY_DIR}")
        message(FATAL_ERROR "Set MAGAZINE_OCCT_ROOT to a complete OCCT SDK")
    endif()

    set(MAGAZINE_OCCT_LINK_LIBRARIES "")
    foreach(occt_library IN LISTS MAGAZINE_OCCT_LIBRARIES)
        find_library(MAGAZINE_OCCT_${occt_library}
            NAMES ${occt_library}
            PATHS "${MAGAZINE_OCCT_LIBRARY_DIR}"
            NO_DEFAULT_PATH
            REQUIRED)
        list(APPEND MAGAZINE_OCCT_LINK_LIBRARIES "${MAGAZINE_OCCT_${occt_library}}")
    endforeach()

    find_library(MAGAZINE_OCCT_TKDESTL
        NAMES TKDESTL
        PATHS "${MAGAZINE_OCCT_LIBRARY_DIR}"
        NO_DEFAULT_PATH)

    add_library(cad_geometry
        src/cad/CadImporter.cpp
        src/cad/CadImporter.hpp
        src/cad/CadPackingRegion.cpp
        src/cad/CadPackingRegion.hpp
    )
    target_include_directories(cad_geometry PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MAGAZINE_OCCT_INCLUDE_DIR})
    target_link_libraries(cad_geometry PUBLIC ${MAGAZINE_OCCT_LINK_LIBRARIES})
    if(MAGAZINE_OCCT_TKDESTL)
        target_compile_definitions(cad_geometry PRIVATE MAGAZINE_HAS_OCCT_STL=1)
        target_link_libraries(cad_geometry PUBLIC "${MAGAZINE_OCCT_TKDESTL}")
    endif()

    add_executable(cad-inspect apps/cad-inspect/main.cpp)
    target_link_libraries(cad-inspect PRIVATE cad_geometry)

    add_executable(cad-capacity apps/cad-capacity/main.cpp)
    target_link_libraries(cad-capacity PRIVATE cad_geometry packing_core)

    if(BUILD_TESTING)
        add_executable(cad-geometry-tests tests/CadGeometryTests.cpp)
        target_link_libraries(cad-geometry-tests PRIVATE cad_geometry packing_core)
        add_test(NAME cad-geometry-tests COMMAND cad-geometry-tests)
    endif()

    add_executable(magazine-capacity-gui WIN32
        apps/magazine-capacity-gui/main.cpp
        apps/magazine-capacity-gui/MainWindow.cpp
        apps/magazine-capacity-gui/MainWindow.hpp
        apps/magazine-capacity-gui/OcctViewport.cpp
        apps/magazine-capacity-gui/OcctViewport.hpp
    )
    target_include_directories(magazine-capacity-gui PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/apps/magazine-capacity-gui
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MAGAZINE_OCCT_INCLUDE_DIR})
    target_link_libraries(magazine-capacity-gui PRIVATE
        packing_core cad_geometry Qt5::Widgets Qt5::Concurrent
        ${MAGAZINE_OCCT_LINK_LIBRARIES})

    foreach(occt_library IN LISTS MAGAZINE_OCCT_RUNTIME_LIBRARIES)
        if(NOT EXISTS "${MAGAZINE_OCCT_RUNTIME_DIR}/${occt_library}.dll")
            message(FATAL_ERROR
                "Missing OCCT runtime DLL: ${MAGAZINE_OCCT_RUNTIME_DIR}/${occt_library}.dll")
        endif()
        add_custom_command(TARGET magazine-capacity-gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${MAGAZINE_OCCT_RUNTIME_DIR}/${occt_library}.dll"
                "$<TARGET_FILE_DIR:magazine-capacity-gui>")
    endforeach()

    foreach(qt_target IN ITEMS Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Concurrent)
        add_custom_command(TARGET magazine-capacity-gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:${qt_target}>"
                "$<TARGET_FILE_DIR:magazine-capacity-gui>")
    endforeach()

    get_target_property(MAGAZINE_QMAKE Qt5::qmake IMPORTED_LOCATION)
    get_filename_component(MAGAZINE_QT_BIN_DIR "${MAGAZINE_QMAKE}" DIRECTORY)
    get_filename_component(MAGAZINE_QT_ROOT_DIR "${MAGAZINE_QT_BIN_DIR}" DIRECTORY)
    foreach(qt_auxiliary_dll IN ITEMS
            libEGL.dll libGLESv2.dll D3Dcompiler_47.dll)
        if(EXISTS "${MAGAZINE_QT_BIN_DIR}/${qt_auxiliary_dll}")
            add_custom_command(TARGET magazine-capacity-gui POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MAGAZINE_QT_BIN_DIR}/${qt_auxiliary_dll}"
                    "$<TARGET_FILE_DIR:magazine-capacity-gui>")
        endif()
    endforeach()
    if(EXISTS "${MAGAZINE_QT_ROOT_DIR}/plugins/platforms")
        add_custom_command(TARGET magazine-capacity-gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${MAGAZINE_QT_ROOT_DIR}/plugins/platforms"
                "$<TARGET_FILE_DIR:magazine-capacity-gui>/platforms")
    endif()
endif()
