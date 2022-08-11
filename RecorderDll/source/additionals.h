#pragma once
#include <pcl/point_types.h>
#include <Windows.h> // For RGBQUAD
#include <chrono>

typedef unsigned char byte;
typedef pcl::PointXYZRGBA PointType;

// Nie miesza napisów z ró¿nych w¹tków.
#define LOG(strText) std::clog << ( std::stringstream() << std::this_thread::get_id() << ": " << strText << "\n" ).str() ;

struct Rgb24 { byte r = 255; byte g = 255; byte b = 255; };
struct Bgr32 { byte b = 255; byte g = 255; byte r = 255; byte _reserved = 0; }; // To¿same z RGBQUAD z Windows.h -> wingdi.h

enum ColorFormat
{
	UnknownFormat = 0,
	RGB_888 = 200, // (red, green, blue) ; 3*8 bitów = 24 bity = 3 bajty
	BGR32 = 400 // RGBQUAD ; (blue, green, red, nieu¿ywane) ; 4*8 bitów = 32 bity = 4 bajty (z czego tylko 3 wykorzystane)
};

struct Colors
{
	Colors()
	{
		Width = 0;
		Height = 0;
		Format = ColorFormat::UnknownFormat;
		Data = nullptr;
	}

	Colors(int width, int height, ColorFormat format, void* data)
	{
		Width = width;
		Height = height;
		Format = format;
		Data = data;
	}

	int Width;
	int Height;
	ColorFormat Format;
	void* Data;
};

class DeviceNotFoundException : public std::runtime_error
{
public:
	DeviceNotFoundException(std::string message)
	: std::runtime_error(message.c_str()) {}

};
