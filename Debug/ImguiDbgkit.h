//
// DebugKit by Wu Haotian, 2023/1/25
//

#ifndef JLUROBOVISION_IMGUIDBGKIT_H
#define JLUROBOVISION_IMGUIDBGKIT_H

#include "imgui.h"
#include <cstdio>
#include <mutex>

#if defined(__linux__)
#include "backends/imgui_impl_sdl.h"
#ifdef IMGUI_CONFIG_FORCE_OPENGL2
#include "backends/imgui_impl_opengl2.h"
#else
#include "backends/imgui_impl_opengl3.h"
#endif
#include <SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif
#elif defined(_WIN32)
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx11.h"
#include <d3d11.h>
#endif

class ImguiDbgkit {

public:
    ImguiDbgkit();
    ~ImguiDbgkit();

    static ImguiDbgkit* get() {
        static std::once_flag once;
        std::call_once(once, [&](){
            instance = new ImguiDbgkit;
#ifdef _WIN32
            instance->WinInit();
#endif
            });
        return instance;
    };
    void NewFrame();
    void FinishFrame();
    bool IsDone() { return done; }

protected:

    ImGuiIO* io;
    bool done;

#if defined(__linux__)
protected:
    SDL_GLContext gl_context;
    SDL_Window* window;
#elif defined(_WIN32)
protected:
	ID3D11Device* g_pd3dDevice = NULL;
	ID3D11DeviceContext* g_pd3dDeviceContext = NULL;
	IDXGISwapChain* g_pSwapChain = NULL;
	ID3D11RenderTargetView* g_mainRenderTargetView = NULL;
    HWND hwnd;
    WNDCLASSEXW wc;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    bool CreateDeviceD3D(HWND hWnd);
    void CleanupDeviceD3D();
    void CreateRenderTarget();
    void CleanupRenderTarget();

    void WinInit();
    static LRESULT WndProcWrapper(HWND a, UINT b, WPARAM c, LPARAM d);
public:
    static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
#else
#error What the fuck
#endif

private:
    static ImguiDbgkit* instance;
};


#endif //JLUROBOVISION_IMGUIDBGKIT_H
