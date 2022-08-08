#include "RecordingManager.h"
#include "KinectV1/KinectV1.h"
#include "KinectV2/KinectV2.h"
#include <chrono>
#include <fstream>

RecordingManager::RecordingManager()
{
	_recorder1 = std::make_shared<KinectV1>();
	_recorder2 = std::make_shared<KinectV2>();
	_colors = new Colors[2];

	// OpóŸnienie, aby kinecty na pewno by³y gotowe (kinect V2 omija³ pierwsz¹ chmurê).
	// Oczywiœcie dobrze by³oby to rozwi¹zaæ w jakiœ cywilizowany sposób.
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

RecordingManager::~RecordingManager()
{
	if (_recording)
		StopRecording();

	if (_recorder1 != nullptr)
	{
		_recorder1->Stop();
		_recorder1 = nullptr;
	}

	if (_recorder2 != nullptr)
	{
		_recorder2->Stop();
		_recorder2 = nullptr;
	}

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
RecordingManager::StartRecording()
{
	_recording = true;
	//_frameNumber = 1;
	
	_recordingThread = std::thread([this]()
	{
		LOG("RecordingManager::StartRecording() - _recordingThread")
		std::chrono::milliseconds interval(1000);
		int frameNumber = 1;
		std::vector<std::string> filenames1;
		std::vector<std::string> filenames2;
		std::string catalog = "./"; // ToDo: trzeba zaimplementowaæ tworzenie katalogu!

		while(_recording)
		{
			auto start = std::chrono::high_resolution_clock::now(); 
			auto target = start + interval;

			// ToDo: Zmierzyæ czas oddzielnie dla ka¿dego podw¹tku.
			// ¯eby wiedzieæ który kinect bardziej zamula 
			// (raczej v1, bo przecie¿ v2 rejestruje chmury ci¹giem (niezale¿nie od ich zapisywania).

			std::thread t1([this, frameNumber, &filenames1, catalog]() {
				std::string filename = "kinectv1-" + std::to_string(frameNumber) + ".pcd";
				_recorder1->RecordOneFrame(catalog + filename);
				filenames1.push_back(filename);
			});

			std::thread t2([this, frameNumber, &filenames2, catalog]() {
				std::string filename = "kinectv2-" + std::to_string(frameNumber) + ".pcd";
				_recorder2->RecordOneFrame(catalog + filename);
				filenames2.push_back(filename);
			});
			
			t1.join();
			t2.join();

			frameNumber++; // Musi byæ po joinach.

			std::this_thread::sleep_until(target);

			auto timeForFrame = std::chrono::high_resolution_clock::now() - start;
			LOG_IMPORTANT( "timeForFrame: " << (timeForFrame.count() / 1000000) << " milliseconds" )
		}

		std::ofstream outfile1;
		outfile1.open(catalog + "kinectv1-settings.vrfilm", std::ios::out | std::ios::trunc );
		outfile1 << "pcd\n";
		outfile1 << std::to_string(filenames1.size()) << '\n';
		for (std::string filename : filenames1) 
			outfile1 << filename << '\n';

		std::ofstream outfile2;
		outfile2.open(catalog + "kinectv2-settings.vrfilm", std::ios::out | std::ios::trunc );
		outfile2 << "pcd\n";
		outfile2 << std::to_string(filenames2.size()) << '\n';
		for (std::string filename : filenames2) 
			outfile2 << filename << '\n';

	});

	// OpóŸnienie, aby na pewno wszystko by³o uruchomione.
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

void
RecordingManager::StopRecording()
{
	_recording = false;
	
	_recordingThread.join();

}
