//
// Created by zhouhb on 23-4-9.
//

#ifndef JLUROBOVISION_DBGGRAPH_H
#define JLUROBOVISION_DBGGRAPH_H

#include "imgui.h"

class DbgGraph {
public:
    DbgGraph(int size);
    ~DbgGraph();
    void NewData(float x);
    void Frame(const char * label, const char * overlay_text, float min, float max, ImVec2 graph_size);

private:

    float * data;
    int offset, size;

};


#endif //JLUROBOVISION_DBGGRAPH_H
