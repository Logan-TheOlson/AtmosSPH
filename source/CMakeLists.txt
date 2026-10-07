cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
set(NAME Stjarna)
project(${NAME})

include(FetchContent)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui
    GIT_TAG        docking
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(volk
    GIT_REPOSITORY https://github.com/zeux/volk
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(volk)

if (NOT IS_DIRECTORY $ENV{VULKAN_SDK})
    message(WARNING "Vulkan SDK not detected, if installed please set VULKAN_SDK env var or manually provide libraries")
else()
    message(STATUS "Found Vulkan SDK in $ENV{VULKAN_SDK}")
endif()

include_directories(SYSTEM $ENV{VULKAN_SDK}/include)

find_package(Vulkan REQUIRED)
set(SDL3_ROOT "C:/SDL3/x86_64-w64-mingw32" CACHE PATH "Root directory of the SDL3 SDK (containing lib/)")
find_library(SDL_LIBRARY NAMES SDL3 HINTS "${SDL3_ROOT}/lib" REQUIRED)
find_program(GLSLC glslc HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin" REQUIRED)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders)
add_custom_command(
    OUTPUT  ${SHADER_OUT}/circle.vert.spv ${SHADER_OUT}/circle.frag.spv
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_OUT}
    COMMAND ${GLSLC} ${SHADER_DIR}/circle.vert -o ${SHADER_OUT}/circle.vert.spv
    COMMAND ${GLSLC} ${SHADER_DIR}/circle.frag -o ${SHADER_OUT}/circle.frag.spv
    DEPENDS ${SHADER_DIR}/circle.vert ${SHADER_DIR}/circle.frag
    VERBATIM
)
add_custom_command(
    OUTPUT  ${SHADER_OUT}/rect.vert.spv ${SHADER_OUT}/rect.frag.spv
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_OUT}
    COMMAND ${GLSLC} ${SHADER_DIR}/rect.vert -o ${SHADER_OUT}/rect.vert.spv
    COMMAND ${GLSLC} ${SHADER_DIR}/rect.frag -o ${SHADER_OUT}/rect.frag.spv
    DEPENDS ${SHADER_DIR}/rect.vert ${SHADER_DIR}/rect.frag
    VERBATIM
)
add_custom_target(Shaders DEPENDS
    ${SHADER_OUT}/circle.vert.spv ${SHADER_OUT}/circle.frag.spv
    ${SHADER_OUT}/rect.vert.spv ${SHADER_OUT}/rect.frag.spv
)

# Karla ships with the fetched imgui source (misc/fonts) — copy it next to the exe so
# VulkanContext::InitImGui can load a real sans-serif font instead of ImGui's default bitmap font.
# Karla's soft, rounded letterforms are the closest bundled match to a Pinterest-style rounded sans.
set(FONT_OUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/fonts)
add_custom_command(
    OUTPUT  ${FONT_OUT}/Karla-Regular.ttf
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FONT_OUT}
    COMMAND ${CMAKE_COMMAND} -E copy ${imgui_SOURCE_DIR}/misc/fonts/Karla-Regular.ttf ${FONT_OUT}/Karla-Regular.ttf
    DEPENDS ${imgui_SOURCE_DIR}/misc/fonts/Karla-Regular.ttf
    VERBATIM
)
add_custom_target(Fonts DEPENDS ${FONT_OUT}/Karla-Regular.ttf)

if (WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WIN32_KHR")
elseif (LINUX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WAYLAND_KHR")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)
target_include_directories(imgui_lib PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${Vulkan_INCLUDE_DIRS}
)
target_compile_definitions(imgui_lib PRIVATE
    IMGUI_IMPL_VULKAN_USE_VOLK
    "IMGUI_IMPL_VULKAN_VOLK_FILENAME=<volk.h>"
    VK_NO_PROTOTYPES
)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(imgui_lib PRIVATE -Wno-stringop-overflow)
endif()
target_link_libraries(imgui_lib PUBLIC volk PRIVATE ${SDL_LIBRARY})

add_executable(${NAME}
    src/main.cpp
    src/engine/Engine.cpp
    src/engine/Object.cpp
    src/engine/Scene.cpp
    src/engine/ScenePresetIO.cpp
    src/renderer/App.cpp
    src/renderer/Window.cpp
    src/renderer/VulkanContext.cpp
    src/ui/Workspace.cpp
    src/ui/EventGizmo.cpp
    src/util/Profiler.cpp
    src/util/ThreadPool.cpp
    src/util/Recorder.cpp
    src/util/DataRecorder.cpp
    src/util/RunNaming.cpp
    src/util/Vector.cpp
    src/util/ColorPattern.cpp
    src/physics/GravityParams.cpp
    src/physics/Quadtree.cpp
    src/physics/Gravity.cpp
    resource.rc
    Stjarna.manifest
)

target_include_directories(${NAME} PRIVATE include)
target_compile_definitions(${NAME} PRIVATE VK_NO_PROTOTYPES)
if (WIN32)
    target_compile_definitions(${NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
set_target_properties(${NAME} PROPERTIES DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(${NAME} PRIVATE cxx_std_20)
target_link_libraries(${NAME} PRIVATE ${SDL_LIBRARY} imgui_lib volk)
add_dependencies(${NAME} Shaders Fonts)

if (MINGW)
    target_link_options(${NAME} PRIVATE -static-libgcc -static-libstdc++ -mwindows)

    # winpthread has to stay a DLL on this toolchain; copy it next to the exe
    get_filename_component(MINGW_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
    add_custom_command(TARGET ${NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MINGW_BIN_DIR}/libwinpthread-1.dll" "$<TARGET_FILE_DIR:${NAME}>"
            VERBATIM
    )
endif ()

add_custom_command(TARGET ${NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL3_ROOT}/bin/SDL3.dll" "$<TARGET_FILE_DIR:${NAME}>"
        VERBATIM
)
