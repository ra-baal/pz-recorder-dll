#pragma once
#include <pcl/point_types.h>
#include <Windows.h> // For RGBQUAD
#include <chrono>

typedef unsigned char byte;
typedef pcl::PointXYZRGBA PointType;
typedef RGBQUAD ColorType;

// Nie miesza napisów z ró¿nych w¹tków.
#define LOG(strText) //std::clog << ( std::stringstream() << std::this_thread::get_id() << ": " << strText << "\n" ).str() ;
#define LOG_IMPORTANT(strText) std::clog << ( std::stringstream() << std::this_thread::get_id() << ": " << strText << "\n" ).str() ;

enum PixelFormat
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
		Format = PixelFormat::UnknownFormat;
		Data = nullptr;
	}

	Colors(int width, int height, PixelFormat format, void* data)
	{
		Width = width;
		Height = height;
		Format = format;
		Data = data;
	}

	int Width;
	int Height;
	PixelFormat Format;
	void* Data;
};

class DeviceNotFoundException : public std::runtime_error
{
public:
	DeviceNotFoundException(std::string message)
	: std::runtime_error(message.c_str()) {}

};


/// <summary>
/// Wszystko publiczne!
/// Ta struktura to przerost formy nad potrzebami.
/// Raczej do wyrzucenia.
/// </summary>
template<typename DataType>
struct ColorPixels
{
	public:
		int _width;
		int _height;
		DataType* _data;

		ColorPixels()
		: _width(0), _height(0), _data(nullptr)
		{
			std::clog << "ColorPixels()" << std::endl;

		}

		ColorPixels(const ColorPixels<ColorPixels>& other) = delete;
		

		/// <summary>
		/// 
		/// </summary>
		/// <param name="bytes">Zostanie wykonana kopia.</param>
		ColorPixels(int width, int height, DataType* src)
		: _width(width), _height(height)
		{
			std::clog << "ColorPixels(int width, int height, DataType* src)" << std::endl;

			int length = _width*_height;
			_data = new DataType[length];

			memcpy(_data, src, length);
		}

		~ColorPixels()
		{
			std::clog << "~ColorPixels()" << std::endl;
			if (_data)
				delete[] _data;
		}


		// todo
		// Usun¹æ albo poprawnie zaimplemntwoaæ operato przypisan
		// ale jeœli i tak bêdzie wskaŸnik wtedy mo¿e byæ nie wa¿ne to.

};



