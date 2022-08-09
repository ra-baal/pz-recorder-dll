#include "RecordingManagerAPI.h"
#include "../IRecordingManager.h"
#include "../RecordingManager.h"

int RecordingManager_Version()
{
    return 220809; // 22-08-09
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
