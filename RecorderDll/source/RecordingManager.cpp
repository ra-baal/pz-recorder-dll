#include "RecordingManager.h"
#include "KinectV1/KinectV1.h"
#include "KinectV2/KinectV2.h"
#include <chrono>

RecordingManager::RecordingManager()
{
	_recorder1 = std::make_shared<KinectV1>();
	_recorder2 = std::make_shared<KinectV2>();
	_colors = new Colors[2];
}

RecordingManager::~RecordingManager()
{
	delete[] _colors;
}

/// ToDo: _recorder1 i _recorder nigdy nie s¹ nullem.
int
RecordingManager::GetRecordersNumber()
{
	if (_recorder1 == nullptr && _recorder2 == nullptr)
		return 0;
	
	if (_recorder1 == nullptr || _recorder2 == nullptr)
		return 1;

	return 2;
}

/// <summary>
/// Color bitmap for each kinect.
/// </summary>
/// <returns>Array of Colors</returns>
Colors*
RecordingManager::GetColorBitmaps()
{
	_colors[0] = _recorder1->GetColorPixels();
	_colors[1] = _recorder2->GetColorPixels();
	return _colors;
}

void
RecordingManager::RecordingMode()
{
	std::chrono::milliseconds interval(1000);
	_recording = true;
	_frameNumber = 1;
	
	_recordingThread = std::thread([this, interval]()
	{
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		while(_recording)
		{
			auto start = std::chrono::high_resolution_clock::now(); 
			auto target = start + interval;

			std::thread t1([this]() {
				_recorder1->RecordOneFrame(std::to_string(_frameNumber) + "-1.pcd");
			});

			std::thread t2([this]() {
				_recorder2->RecordOneFrame(std::to_string(_frameNumber) + "-2.pcd");
			});

			t1.join();
			t2.join();

			_frameNumber++; // Musi byæ po joinach.

			std::this_thread::sleep_until(target);
		}

	});

}

void
RecordingManager::PreviewMode()
{
	_recording = false;
	_recordingThread.join();
}
