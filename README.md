# easyexif

A tiny ISO-compliant C++ EXIF parsing library.

EasyEXIF is a tiny, lightweight C++ library that parses basic information out of JPEG files. It uses only the std::string library and is otherwise pure C++. You pass it the binary contents of a JPEG file, and it parses several of the most important EXIF fields for you.

Why use this library? **Include one .h file, compile one .cpp file, and that's it.**

Sometimes you just need to quickly extract basic information from a JPEG file's EXIF headers: the time the image was taken (not the file timestamp, the camera's internal time), the F-stop or exposure time, GPS information embedded in the EXIF file, what the camera make and model was, etc.. Unfortunately, all the EXIF libraries out there are not very lightweight or easy to integrate into larger programs. EasyEXIF aims to solve that problem, and is released under a very liberal BSD License for use practically anywhere.

The only two files you need for your project are `exif.cpp` and `exif.h`. There are no dependencies on any build system or external libraries.

Here is an up-to-date list of [valid EXIF tags](https://exiftool.org/TagNames/EXIF.html).

## Fork

[asmaloney](https://github.com/asmaloney) forked this library [from here](https://github.com/mayanklahiri/easyexif) since it seemed to no longer be maintained. I have made the following changes:

- fixed a couple of bugs
- fixed several static analysis findings
- added several fields
- set defaults according to the standard
- added a way to determine if a field was set in the image's EXIF data or not
- added a CMake file
- added a clang-format file
- added CI for macOS, Linux, and Windows
- added CI for clang-format
- now requires C++17

## Features:

1. Supports common EXIF fields including GPS, ISO speed, etc.
2. Extensively documented in the source.
3. Valgrind tested for memory leaks.
4. Handles corrupt JPEGs.
5. Compiles without complaints using `-Wall -Wextra -Werror -pedantic -ansi` on gcc v4.8.2
6. No uses of new/malloc.

## License

BSD. Free for personal and commercial use.

## Contributions

Reasonable pull requests are gladly accepted. Please ensure that you have run the following steps before submitting a patch:

1. `clang-format` the source files, using the default style (Google).
2. Run `test.sh` to make sure test images are being parsed correctly.
3. Compile and run `demo` using `valgrind --leak-check=full` to ensure that there are no memory leaks.
4. Add yourself to the contributors list in `CONTRIBUTORS`.
5. For misbehaving images, please try to include a test image that misbehaves in the `test-images` folder.

## Example:

See `demo.cpp` for other EXIF fields.

```C++
  #include "exif.h"

  EXIFInfo result;
  result.parseFrom(JPEGFileBuffer, BufferSize);

  printf("Camera make       : %s\n", result.Make.c_str());
  printf("Camera model      : %s\n", result.Model.c_str());
  printf("Software          : %s\n", result.Software.c_str());
  printf("Bits per sample   : %d\n", result.BitsPerSample);
  printf("Image width       : %d\n", result.ImageWidth);
  printf("Image height      : %d\n", result.ImageHeight);
  printf("Image description : %s\n", result.ImageDescription.c_str());
  printf("Image orientation : %d\n", result.Orientation);
  printf("Image copyright   : %s\n", result.Copyright.c_str());
  printf("Image date/time   : %s\n", result.DateTime.c_str());
  printf("Original date/time: %s\n", result.DateTimeOriginal.c_str());
  printf("Digitize date/time: %s\n", result.DateTimeDigitized.c_str());
  printf("Subsecond time    : %s\n", result.SubSecTimeOriginal.c_str());
  printf("Exposure time     : 1/%d s\n", (unsigned) (1.0/result.ExposureTime));
  printf("F-stop            : f/%.1f\n", result.FNumber);
  printf("ISO speed         : %d\n", result.ISOSpeedRatings);
  printf("Subject distance  : %f m\n", result.SubjectDistance);
  printf("Exposure bias     : %f EV\n", result.ExposureBiasValue);
  printf("Flash used?       : %d\n", result.Flash);
  printf("Metering mode     : %d\n", result.MeteringMode);
  printf("Lens focal length : %f mm\n", result.FocalLength);
  printf("35mm focal length : %u mm\n", result.FocalLengthIn35mm);
  printf("GPS Latitude      : %f deg\n", result.GeoLocation.Latitude);
  printf("GPS Longitude     : %f deg\n", result.GeoLocation.Longitude);
  printf("GPS Altitude      : %f m\n", result.GeoLocation.Altitude);
```
