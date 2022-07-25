#pragma once
#include <pcl/point_types.h>
#include <Windows.h> // For RGBQUAD

typedef unsigned char byte;
typedef pcl::PointXYZRGBA PointType;
typedef RGBQUAD ColorType;

struct Colors
{
	int Width;
	int Height;
	ColorType* Data;
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
		// Usun�� albo poprawnie zaimplemntwoa� operato przypisan
		// ale je�li i tak b�dzie wska�nik wtedy mo�e by� nie wa�ne to.

};



