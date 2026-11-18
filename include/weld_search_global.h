#ifndef WELD_SEARCH_GLOBAL_H
#define WELD_SEARCH_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(WELD_SEARCH_LIBRARY)
#define WELD_SEARCH_EXPORT Q_DECL_EXPORT
#else
#define WELD_SEARCH_EXPORT Q_DECL_IMPORT
#endif

#endif // WELD_SEARCH_GLOBAL_H
