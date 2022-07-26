#pragma once

#include "IRecordingManager.h"
#include "ICloudRecorder.h"

class RecordingManager : IRecordingManager
{
public:
	RecordingManager();
	// Inherited via IRecordingManager
	// virtual RecorderState GetState() override;
	virtual int GetRecordersNumber() override;
    virtual Colors GetColorBitmap() override;
	virtual void RecordingMode() override;
	virtual void PreviewMode() override;

protected:
	std::shared_ptr<ICloudRecorder> _recorder1;
	std::shared_ptr<ICloudRecorder> _recorder2;

};