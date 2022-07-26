#include "RecordingManagerAPI.h"
#include "../IRecordingManager.h"
#include "../RecordingManager.h"

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

Colors RecordingManager_GetColorBitmap(void* recordingManager)
{
    return ((IRecordingManager*)recordingManager)->GetColorBitmap();
}

void RecordingManager_RecordingMode(void* recordingManager)
{
    ((IRecordingManager*)recordingManager)->RecordingMode();
}

void RecordingManager_PreviewMode(void* recordingManager)
{
    ((IRecordingManager*)recordingManager)->PreviewMode();
}
