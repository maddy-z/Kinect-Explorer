#include "MainWnd.h"

#include <QtGui/QApplication>

int main ( int argc, char * argv[] )
{
	QApplication app ( argc, argv );

	MainWnd w;
	w.show ();

	return app.exec ();
}