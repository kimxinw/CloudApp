#ifndef CLOUD_SELVE_GLOBAL_H
#define CLOUD_SELVE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CLOUD_SELVE_LIBRARY)
#define CLOUD_SELVE_EXPORT Q_DECL_EXPORT
#else
#define CLOUD_SELVE_EXPORT Q_DECL_IMPORT
#endif

#endif // CLOUD_SELVE_GLOBAL_H
