//
// Created by zhouhb on 23-4-9.
//

#include "DbgGraph.h"

DbgGraph::DbgGraph(int size) {
    data = new float[size];
    this->size = size;
}

DbgGraph::~DbgGraph() {
    delete[] data;
}

void DbgGraph::NewData(float x) {
    data[offset] = x;
    offset = (offset + 1) % size;
}

void DbgGraph::Frame(const char * label, const char * overlay_text, float min, float max, ImVec2 graph_size) {
    ImGui::PlotLines(label,
                     data,
                     size,
                     offset,
                     overlay_text,
                     min,
                     max,
                     graph_size);
}
