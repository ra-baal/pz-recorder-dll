#pragma once
    
#ifdef RecorderDll_EXPORTS
#define RecorderDll_API __declspec(dllexport)
#else
#define RecorderDll_API __declspec(dllimport)
#endif

#include "../additionals.h"

extern "C" RecorderDll_API int RecordingManager_Test(int a, int b);
extern "C" RecorderDll_API void* RecordingManager_New();
extern "C" RecorderDll_API void RecordingManager_Delete(void* recordingManager);
extern "C" RecorderDll_API int RecordingManager_GetRecordersNumber(void* recordingManager);
extern "C" RecorderDll_API Colors* RecordingManager_GetColorBitmaps(void* recordingManager);
extern "C" RecorderDll_API void RecordingManager_StartRecording(void* recordingManager);
extern "C" RecorderDll_API void RecordingManager_StopRecording(void* recordingManager);
