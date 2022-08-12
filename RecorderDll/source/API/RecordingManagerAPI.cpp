#include "RecordingManagerAPI.h"
#include "../RecordingManager/IRecordingManager.h"
#include "../RecordingManager/RecordingManager.h"

#define DLL_VERSION 220812 // 22-08-12

int RecordingManager_Version()
{
    return DLL_VERSION;
}

int RecordingManager_Test(int a, int b)
{
    return a + b;
}

void* RecordingManager_New()
{
    // Mo¿na "wstrzykn¹æ" inn¹ klasê implementuj¹c¹ interfejs.
    return (IRecordingManager*)(new RecordingManager());
}

void RecordingManager_Delete(void* recordingManager)
{
    delete ((IRecordingManager*)recordingManager);
}

int RecordingManager_GetRecordersNumber(void* recordingManager)
{
    return ((IRecordingManager*)recordingManager)->GetRecordersNumber();
}

Colors* RecordingManager_GetColorBitmaps(void* recordingManager)
{
    return ((IRecordingManager*)recordingManager)->GetColorBitmaps();
}

void RecordingManager_StartRecording(void* recordingManager)
{
    ((IRecordingManager*)recordingManager)->StartRecording();
}

void RecordingManager_StopRecording(void* recordingManager)
{
    ((IRecordingManager*)recordingManager)->StopRecording();
}

void RecordingManager_SetDirectory(void* recordingManager, const char* str)
{
    ((IRecordingManager*)recordingManager)->SetDirectory(str);
}
