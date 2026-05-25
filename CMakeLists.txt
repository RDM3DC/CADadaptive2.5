cmake_minimum_required(VERSION 3.24)

project(AdaptiveCAD
    VERSION 0.1.0
    DESCRIPTION "AdaptiveCAD kernel skeleton"
    LANGUAGES CXX)

include(GNUInstallDirs)

option(ADAPTIVECAD_BUILD_TESTS "Build unit tests" ON)
option(ADAPTIVECAD_BUILD_CLI "Build CLI demo" ON)
option(ADAPTIVECAD_BUILD_UI "Build Win32 UI demo" ON)
option(ADAPTIVECAD_ENABLE_OPENCASCADE "Enable OpenCascade-backed geometry adapter when available" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runbin")

configure_file(
    include/adaptivecad/Version.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/adaptivecad/Version.hpp"
    @ONLY
)

function(adaptivecad_enable_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(adaptivecad_core
    src/core/AdaptiveField.cpp
    src/core/InverseRecovery.cpp
)

add_library(adaptivecad_geometry
    src/geometry/BRepKernel.cpp
    src/geometry/HistoryAwareKernel.cpp
    src/geometry/StubGeometryAdapter.cpp
    src/geometry/OpenCascadeGeometryAdapter.cpp
)

add_library(adaptivecad_tool
    src/tool/MetamaterialScaffold.cpp
    src/tool/SceneDocument.cpp
    src/tool/SessionBundleIO.cpp
)

target_include_directories(adaptivecad_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(adaptivecad_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(adaptivecad_tool
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(adaptivecad_tool
    PUBLIC
        adaptivecad_core
        adaptivecad_geometry
)

if(ADAPTIVECAD_ENABLE_OPENCASCADE)
    find_package(OpenCASCADE QUIET)
    if(OpenCASCADE_FOUND)
        target_compile_definitions(adaptivecad_geometry PRIVATE ADAPTIVECAD_HAS_OPENCASCADE=1)
        target_compile_definitions(adaptivecad_tool PRIVATE ADAPTIVECAD_HAS_OPENCASCADE=1)

        if(DEFINED OpenCASCADE_INCLUDE_DIR)
            target_include_directories(adaptivecad_geometry PRIVATE ${OpenCASCADE_INCLUDE_DIR})
            target_include_directories(adaptivecad_tool PRIVATE ${OpenCASCADE_INCLUDE_DIR})
        endif()

        if(DEFINED OpenCASCADE_LIBRARIES)
            target_link_libraries(adaptivecad_geometry PRIVATE ${OpenCASCADE_LIBRARIES})
            target_link_libraries(adaptivecad_tool PRIVATE ${OpenCASCADE_LIBRARIES})
        endif()

        message(STATUS "OpenCascade adapter enabled")
    else()
        message(WARNING "ADAPTIVECAD_ENABLE_OPENCASCADE=ON but OpenCASCADE was not found; using stub adapter fallback.")
    endif()
endif()

adaptivecad_enable_warnings(adaptivecad_core)
adaptivecad_enable_warnings(adaptivecad_geometry)
adaptivecad_enable_warnings(adaptivecad_tool)

if(ADAPTIVECAD_BUILD_CLI)
    add_executable(adaptivecad_cli src/app/main.cpp)
    target_link_libraries(adaptivecad_cli PRIVATE adaptivecad_core adaptivecad_geometry adaptivecad_tool)
    adaptivecad_enable_warnings(adaptivecad_cli)
    set_target_properties(adaptivecad_cli PROPERTIES OUTPUT_NAME "adaptivecad")
endif()

if(WIN32 AND ADAPTIVECAD_BUILD_UI)
    add_executable(adaptivecad_ui WIN32 src/ui/main_win32.cpp)
    target_link_libraries(adaptivecad_ui PRIVATE adaptivecad_core adaptivecad_geometry adaptivecad_tool comctl32)
    adaptivecad_enable_warnings(adaptivecad_ui)
    set_target_properties(adaptivecad_ui PROPERTIES
        OUTPUT_NAME "cad_workbench_live"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/runbin"
    )
endif()

if(ADAPTIVECAD_BUILD_TESTS)
    enable_testing()

    add_executable(adaptivecad_tests tests/test_adaptive_field.cpp)
    target_link_libraries(adaptivecad_tests PRIVATE adaptivecad_core)
    adaptivecad_enable_warnings(adaptivecad_tests)

    add_test(NAME adaptivecad_tests COMMAND adaptivecad_tests)

    add_executable(adaptivecad_geometry_tests tests/test_geometry_adapter.cpp)
    target_link_libraries(adaptivecad_geometry_tests PRIVATE adaptivecad_geometry)
    adaptivecad_enable_warnings(adaptivecad_geometry_tests)

    add_test(NAME adaptivecad_geometry_tests COMMAND adaptivecad_geometry_tests)

    add_executable(adaptivecad_brep_tests tests/test_brep_kernel.cpp)
    target_link_libraries(adaptivecad_brep_tests PRIVATE adaptivecad_geometry)
    adaptivecad_enable_warnings(adaptivecad_brep_tests)

    add_test(NAME adaptivecad_brep_tests COMMAND adaptivecad_brep_tests)

    add_executable(adaptivecad_history_tests tests/test_history_aware_kernel.cpp)
    target_link_libraries(adaptivecad_history_tests PRIVATE adaptivecad_geometry)
    adaptivecad_enable_warnings(adaptivecad_history_tests)

    add_test(NAME adaptivecad_history_tests COMMAND adaptivecad_history_tests)

    add_executable(adaptivecad_tool_tests tests/test_scene_document.cpp)
    target_link_libraries(adaptivecad_tool_tests PRIVATE adaptivecad_tool)
    adaptivecad_enable_warnings(adaptivecad_tool_tests)

    add_test(NAME adaptivecad_tool_tests COMMAND adaptivecad_tool_tests)

    add_executable(adaptivecad_scaffold_tests tests/test_metamaterial_scaffold.cpp)
    target_link_libraries(adaptivecad_scaffold_tests PRIVATE adaptivecad_tool)
    adaptivecad_enable_warnings(adaptivecad_scaffold_tests)

    add_test(NAME adaptivecad_scaffold_tests COMMAND adaptivecad_scaffold_tests)

    if(TARGET adaptivecad_cli)
        add_test(NAME adaptivecad_cli_version COMMAND adaptivecad_cli --version)
        set_tests_properties(adaptivecad_cli_version PROPERTIES
            PASS_REGULAR_EXPRESSION "AdaptiveCAD ${PROJECT_VERSION}")

        add_test(NAME adaptivecad_cli_help COMMAND adaptivecad_cli --help)
        set_tests_properties(adaptivecad_cli_help PROPERTIES
            PASS_REGULAR_EXPRESSION "Usage: adaptivecad")

        add_test(NAME adaptivecad_cli_export_geometry
            COMMAND adaptivecad_cli
                --export-geometry "${CMAKE_CURRENT_SOURCE_DIR}/samples/simple_wedge.stl"
                --output "${CMAKE_CURRENT_BINARY_DIR}/simple_wedge_export.obj"
                --healing strict)
        set_tests_properties(adaptivecad_cli_export_geometry PROPERTIES
            PASS_REGULAR_EXPRESSION "Exported geometry")

        add_test(NAME adaptivecad_cli_inspect_geometry_json
            COMMAND adaptivecad_cli
                --inspect-geometry "${CMAKE_CURRENT_SOURCE_DIR}/samples/simple_wedge.stl"
                --healing strict
                --json)
        set_tests_properties(adaptivecad_cli_inspect_geometry_json PROPERTIES
            PASS_REGULAR_EXPRESSION "source_kind.*stl_mesh")

        add_test(NAME adaptivecad_cli_create_primitive
            COMMAND adaptivecad_cli
                --create-primitive torus
                --major-radius 1.2
                --minor-radius 0.2
                --major-segments 12
                --minor-segments 8
                --output "${CMAKE_CURRENT_BINARY_DIR}/torus_primitive.obj"
                --healing strict)
        set_tests_properties(adaptivecad_cli_create_primitive PROPERTIES
            PASS_REGULAR_EXPRESSION "Created primitive")

        add_test(NAME adaptivecad_cli_create_scaffold
            COMMAND adaptivecad_cli
                --create-primitive scaffold
                --cells-x 2
                --cells-y 1
                --cells-z 1
                --spacing 0.75
                --damage-radius 0.10
                --output "${CMAKE_CURRENT_BINARY_DIR}/scaffold_primitive.obj"
                --healing strict)
        set_tests_properties(adaptivecad_cli_create_scaffold PROPERTIES
            PASS_REGULAR_EXPRESSION "Created primitive")
    endif()
endif()

install(TARGETS adaptivecad_core adaptivecad_geometry adaptivecad_tool
    EXPORT AdaptiveCADTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

if(TARGET adaptivecad_cli)
    install(TARGETS adaptivecad_cli RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

if(TARGET adaptivecad_ui)
    install(TARGETS adaptivecad_ui RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

install(DIRECTORY include/adaptivecad DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.hpp")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/adaptivecad/Version.hpp"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/adaptivecad")
install(DIRECTORY samples DESTINATION "${CMAKE_INSTALL_DATADIR}/adaptivecad")
install(FILES README.md COMMERCIALIZATION.md adaptive_cad_flat_adasccsc.md
    DESTINATION "${CMAKE_INSTALL_DATADIR}/adaptivecad")

set(CPACK_PACKAGE_NAME "AdaptiveCAD")
set(CPACK_PACKAGE_VENDOR "AdaptiveCAD")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "AdaptiveCAD")
set(CPACK_GENERATOR "ZIP")
include(CPack)
