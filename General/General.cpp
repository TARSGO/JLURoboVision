
#include "General.h"
#include <string>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::minutes;
using std::chrono::seconds;
using std::to_string;

std::string CurrentPreciseTime() {
    static auto begin = high_resolution_clock::now();
    
    auto preciseTime = high_resolution_clock::now() - begin;
    auto microsec = duration_cast<microseconds>(preciseTime).count();
    auto mins = duration_cast<minutes>(preciseTime).count();
    auto secs = duration_cast<seconds>(preciseTime).count();

    microsec -= secs * 1000000;
    secs -= mins * 60;

    std::string ret;
    ret += to_string(mins) + ":"
         + to_string(secs) + "."
         + to_string(microsec);

    return ret;
}

// https://codereview.stackexchange.com/a/165162
std::ostream& HexDump(std::ostream& os, const void *buffer,
                       std::size_t bufsize, bool showPrintableChars)
{
    if (buffer == nullptr) {
        return os;
    }
    auto oldFormat = os.flags();
    auto oldFillChar = os.fill();
    constexpr std::size_t maxline{8};
    // create a place to store text version of string
    char renderString[maxline+1];
    char *rsptr{renderString};
    // convenience cast
    const unsigned char *buf{reinterpret_cast<const unsigned char *>(buffer)};

    for (std::size_t linecount=maxline; bufsize; --bufsize, ++buf) {
        os << std::setw(2) << std::setfill('0') << std::hex
           << static_cast<unsigned>(*buf) << ' ';
        *rsptr++ = std::isprint(*buf) ? *buf : '.';
        if (--linecount == 0) {
            *rsptr++ = '\0';  // terminate string
            if (showPrintableChars) {
                os << " | " << renderString;
            }
            os << '\n';
            rsptr = renderString;
            linecount = std::min(maxline, bufsize);
        }
    }
    // emit newline if we haven't already
    if (rsptr != renderString) {
        if (showPrintableChars) {
            for (*rsptr++ = '\0'; rsptr != &renderString[maxline+1]; ++rsptr) {
                os << "   ";
            }
            os << " | " << renderString;
        }
        os << '\n';
    }

    os.fill(oldFillChar);
    os.flags(oldFormat);
    return os;
}

std::string CurrentDateTime() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();

    return str;
}

//用于比较预测欧拉角和实际欧拉角，留着或许有用
void Debug_YawPitchVisualizer(double currentYaw, double currentPitch, double targetYaw, double targetPitch, bool targetValid) {
    ImGui::Begin("Yaw pitch debugging");
    ImDrawList *drawList = ImGui::GetWindowDrawList();

    // Draw Yaw
    ImVec2 canvasP0 = ImGui::GetCursorScreenPos(), canvasSize = ImGui::GetContentRegionAvail();
    ImVec2 canvasP1(canvasP0.x + canvasSize.x, canvasP0.y + canvasSize.y);
    drawList->PushClipRect(canvasP0, canvasP1, true);
    canvasSize.y = 30;
    canvasP1.y = canvasP0.y + canvasSize.y;
    ImVec2 canvasCenter((canvasP0.x + canvasP1.x) / 2, (canvasP0.y + canvasP1.y) / 2);

    // Yaw Axis
    drawList->AddLine({1, canvasP0.y}, {1, canvasP1.y}, IM_COL32_WHITE);
    drawList->AddLine({canvasP1.x - 1, canvasP0.y}, {canvasP1.x - 1, canvasP1.y}, IM_COL32_WHITE);
    drawList->AddLine({0, canvasCenter.y}, {canvasP1.x, canvasCenter.y}, IM_COL32_WHITE);
    drawList->AddLine({canvasCenter.x, canvasCenter.y - 8}, {canvasCenter.x, canvasCenter.y + 8}, IM_COL32_WHITE);
    // Target (GREEN)
    float targetCanvasX = (targetYaw / 180.0 + 1) * canvasCenter.x;
    if (targetValid)
        drawList->AddLine({targetCanvasX, canvasCenter.y - 10}, {targetCanvasX, canvasCenter.y + 10}, IM_COL32(0, 255, 0 ,255));
    // Current (RED)
    float currentCanvasX = (currentYaw / 180.0 + 1) * canvasCenter.x;
    drawList->AddLine({currentCanvasX, canvasCenter.y - 10}, {currentCanvasX, canvasCenter.y + 10}, IM_COL32(255, 0, 0 ,255));

    // Pitch Axis
    canvasP0.y += 30; canvasSize = ImGui::GetContentRegionAvail(); canvasSize.y -= 30; canvasSize.y -= 5;
    canvasP1.y = canvasP0.y + canvasSize.y;
    canvasCenter = {(canvasP0.x + canvasP1.x) / 2, (canvasP0.y + canvasP1.y) / 2};

    drawList->AddLine({canvasCenter.x - 30, canvasP0.y}, {canvasCenter.x + 30, canvasP0.y}, IM_COL32_WHITE);
    drawList->AddLine({canvasCenter.x - 30, canvasP1.y}, {canvasCenter.x + 30, canvasP1.y}, IM_COL32_WHITE);
    drawList->AddLine({canvasCenter.x, canvasP0.y}, {canvasCenter.x, canvasP1.y}, IM_COL32_WHITE);
    drawList->AddLine({canvasCenter.x - 8, canvasCenter.y}, {canvasCenter.x + 8, canvasCenter.y}, IM_COL32_WHITE);
    // Target (GREEN)
    float targetCanvasY = -(targetPitch / 30.0 + 1) * (canvasP1.y - canvasCenter.y) + canvasP1.y;
    if (targetValid)
        drawList->AddLine({canvasCenter.x - 10, targetCanvasY}, {canvasCenter.x + 10, targetCanvasY}, IM_COL32(0, 255, 0, 255));
    // Current (RED)
    float currentCanvasY = -(currentPitch / 30.0 + 1) * (canvasP1.y - canvasCenter.y) + canvasP1.y;
    drawList->AddLine({canvasCenter.x - 10, currentCanvasY}, {canvasCenter.x + 10, currentCanvasY}, IM_COL32(255, 0 , 0, 255));


    drawList->PopClipRect();
    ImGui::End();
}
