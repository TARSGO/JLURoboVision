file(GLOB IMGUI_SOURCE imgui/*.h imgui/*.cpp)
file(GLOB IMGUIZMO_SOURCE ImGuizmo/*.cpp ImGuizmo/*.h)
set(IMGUI_SOURCE
		${IMGUI_SOURCE}
		${IMGUIZMO_SOURCE}
		Dbg3DScene.cpp
                Dbg3DScene.h
)


# Use OpenGL2 if this option is set. Otherwise defaults to OpenGL3.
set(IMGUI_USE_OPENGL2 OFF)
message("Current source dir ${CMAKE_CURRENT_SOURCE_DIR}")
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/UseOpenGL2)
    set(IMGUI_USE_OPENGL2 ON)
endif()

if(UNIX)
    include(FindPkgConfig)
    pkg_search_module(SDL2 REQUIRED sdl2)
    message("SDL2 inc: ${SDL2_INCLUDE_DIRS}")
    include_directories(${SDL2_INCLUDE_DIRS})

    set(IMGUI_SOURCE
            ${IMGUI_SOURCE}
            imgui/backends/imgui_impl_sdl.cpp
            imgui/backends/imgui_impl_sdl.h
    )
    if(IMGUI_USE_OPENGL2)
        add_definitions(-DIMGUI_CONFIG_FORCE_OPENGL2)
        set(IMGUI_SOURCE
                ${IMGUI_SOURCE}
                ImguiDbgkit_OpenGL2.cpp
                imgui/backends/imgui_impl_opengl2.cpp
                imgui/backends/imgui_impl_opengl2.h
            )
    else()
        set(IMGUI_SOURCE
                ${IMGUI_SOURCE}
                ImguiDbgkit.cpp
                imgui/backends/imgui_impl_opengl3.cpp
                imgui/backends/imgui_impl_opengl3.h
            )
    endif()

    set(IMGUI_LINKOPT
            ${SDL2_LIBRARIES}
            -lGL
    )
elseif(WIN32)
	
    include("../windows_libs.cmake")
    include(${WIN_OPENCV_CONFIG})
    include(${WIN_EIGEN3_CONFIG})
    include(${EIGEN3_USE_FILE})
	
	include_directories(${OpenCV_INCLUDE_DIRS})
	include_directories(${EIGEN3_INCLUDE_DIRS})
	message("ImguiDbgkit: CV: ${OpenCV_INCLUDE_DIRS} Eigen3: ${EIGEN3_INCLUDE_DIRS}")

	set(IMGUI_SOURCE
		${IMGUI_SOURCE}
		ImguiDbgkit_Dx11.cpp
		imgui/backends/imgui_impl_dx11.cpp
		imgui/backends/imgui_impl_dx11.h
		imgui/backends/imgui_impl_win32.cpp
		imgui/backends/imgui_impl_win32.h)
	set(IMGUI_LINKOPT
		d3d11.lib
	)
endif()

include_directories(imgui)
include_directories(imGuIZMO.quat)

add_library(ImguiDbgkit STATIC
		${IMGUI_SOURCE}
		ImguiDbgkit.h
		DbgGraph.cpp
		DbgGraph.h
		)

target_link_libraries(ImguiDbgkit -ldl ${IMGUI_LINKOPT})
