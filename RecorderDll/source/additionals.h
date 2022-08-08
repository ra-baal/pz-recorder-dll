#pragma once
#include <pcl/point_types.h>
#include <Windows.h> // For RGBQUAD
#include <chrono>

typedef unsigned char byte;
typedef pcl::PointXYZRGBA PointType;
typedef RGBQUAD ColorType;

// Nie miesza napisów z ró¿nych w¹tków.
#define LOG(strText) //std::clog << ( std::stringstream() << std::this_thread::get_id() << ": " << strText << "\n" ).str() ;
#define LOG_IMPORTANT(strText) //std::clog << ( std::stringstream() << std::this_thread::get_id() << ": " << strText << "\n" ).str() ;

struct Colors
{
	int Width = 0;
	int Height = 0;
	ColorType* Data = nullptr;
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



