#include "mainwindow.h"
#include <QApplication>

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	MainWindow main_window;
	main_window.show();

	while (!main_window.wasStopped())
	{
		main_window.spinOnce();
	}

	return 0;
}
