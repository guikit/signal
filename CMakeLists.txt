cmake_minimum_required(VERSION 3.21)
project(guikit_signal VERSION 1.0.0 LANGUAGES CXX)

if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(GUIKIT_SIGNAL_BUILD_TESTS "Build GuiKit Signal unit tests" ${PROJECT_IS_TOP_LEVEL})
option(GUIKIT_SIGNAL_INSTALL "Enable CMake install targets for guikit_signal" ${PROJECT_IS_TOP_LEVEL})

if (GUIKIT_SIGNAL_BUILD_TESTS)
    enable_testing()
endif()

if (NOT TARGET guikit::core)
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/guikit/CMakeLists.txt")
        add_subdirectory(third_party/guikit EXCLUDE_FROM_ALL)
    else()
        find_package(guikit REQUIRED)
    endif()
endif()

add_library(guikit_signal STATIC
    src/guikit/signal/signal_view.cpp
    src/guikit/signal/sonar_view.cpp
    src/guikit/signal/waterfall_view.cpp
)
add_library(guikit::signal ALIAS guikit_signal)
set_target_properties(guikit_signal PROPERTIES EXPORT_NAME signal)

target_compile_features(guikit_signal PUBLIC cxx_std_20)
target_include_directories(guikit_signal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(guikit_signal PUBLIC guikit::core)

if (COMMAND guikit_enforce_guidelines)
    guikit_enforce_guidelines(guikit_signal
        HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
        STANDALONE_HEADERS ${GUIKIT_SIGNAL_BUILD_TESTS}
        TEST_NAME guikit_signal_header_purity_scan
    )
endif()

if (GUIKIT_SIGNAL_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(TARGETS guikit_signal
        EXPORT guikitSignalTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/guikit"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    )
    install(EXPORT guikitSignalTargets
        FILE guikitSignalTargets.cmake
        NAMESPACE guikit::
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_signal"
    )
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/guikit_signalConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_signalConfig.cmake"
        INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_signal"
    )
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_signalConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_signalConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/guikit_signalConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guikit_signal"
    )
endif()

if (GUIKIT_SIGNAL_BUILD_TESTS AND TARGET guikit::guikit)
    add_executable(test_signal
        tests/test_signal.cpp
    )
    target_link_libraries(test_signal PRIVATE guikit::signal guikit::guikit)
    if (TARGET SDL3::SDL3-static)
        target_link_libraries(test_signal PRIVATE SDL3::SDL3-static)
    elseif (TARGET SDL3::SDL3)
        target_link_libraries(test_signal PRIVATE SDL3::SDL3)
    endif()
    if (COMMAND guikit_enforce_guidelines)
        guikit_enforce_guidelines(test_signal ALLOW_HEADERS "SDL3/SDL.h")
    endif()
    add_test(NAME signal COMMAND test_signal)
endif()
