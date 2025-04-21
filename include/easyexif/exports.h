
#ifndef EASYEXIF_EXPORT_H
#define EASYEXIF_EXPORT_H

#ifdef EASYEXIF_STATIC_DEFINE
#  define EASYEXIF_EXPORT
#  define EASYEXIF_NO_EXPORT
#else
#  ifndef EASYEXIF_EXPORT
#    ifdef easyexif_EXPORTS
        /* We are building this library */
#      define EASYEXIF_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define EASYEXIF_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef EASYEXIF_NO_EXPORT
#    define EASYEXIF_NO_EXPORT 
#  endif
#endif

#ifndef EASYEXIF_DEPRECATED
#  define EASYEXIF_DEPRECATED __declspec(deprecated)
#endif

#ifndef EASYEXIF_DEPRECATED_EXPORT
#  define EASYEXIF_DEPRECATED_EXPORT EASYEXIF_EXPORT EASYEXIF_DEPRECATED
#endif

#ifndef EASYEXIF_DEPRECATED_NO_EXPORT
#  define EASYEXIF_DEPRECATED_NO_EXPORT EASYEXIF_NO_EXPORT EASYEXIF_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef EASYEXIF_NO_DEPRECATED
#    define EASYEXIF_NO_DEPRECATED
#  endif
#endif

#endif /* EASYEXIF_EXPORT_H */
