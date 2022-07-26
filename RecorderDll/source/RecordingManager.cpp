#include "RecordingManager.h"
#include "KinectV2/KinectV2.h"

RecordingManager::RecordingManager()
{
	//_recorder1 = std::make_shared<KinectV1>();
	_recorder2 = std::make_shared<KinectV2>();
}

int
RecordingManager::GetRecordersNumber()
{
	return _recorder2 == nullptr ? 0 : 1;
}

Colors
RecordingManager::GetColorBitmap()
{
	return _recorder2->GetColorPixels();
}

void
RecordingManager::RecordingMode()
{
	
}

void
RecordingManager::PreviewMode()
{

}
