#pragma once
#include "RecorderState.h"
#include "additionals.h"

class IRecordingManager
{
    public:
        //FindRecorders() ??
        //virtual RecorderState GetState() = 0;
        virtual int GetRecordersNumber() = 0;
        virtual Colors* GetColorBitmaps() = 0;
        virtual void StartRecording() = 0;
        virtual void StopRecording() = 0;
};
