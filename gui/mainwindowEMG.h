/*
 * Gui.h
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#ifndef GUI_H_
#define GUI_H_

#include <QMainWindow>
#include <QTimer>
#include "qcustomplot.h"
#include "ui_mainwindow.h"
#include <sstream>
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QMainWindow>
#include <QDockWidget>
#include <QScrollArea>
#include <QBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QSize>
#include <QCheckBox>
#include <QGraphicsScene>
#include <iostream>
#include <QSplitter>
#include <QPushButton>
#include "myGLWidget.h"
#include "SyncToolsIK.h"
#include <QTreeWidget>
#include "SyncToolsEMG.h"

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer
#include <boost/timer/timer.hpp>

using namespace gl;
namespace Ui
{
	class MainWindow;
}

class MainWindow: public QMainWindow
{
		Q_OBJECT
	public:
		MainWindow ( double dt, std::vector<std::string> dofNames, const std::string modelFileName, std::vector<std::string> channelNames, QWidget* parent = 0 );
		virtual ~MainWindow();
		inline void setTorque ( std::vector <double> torque )
		{
			_torqueGui = torque;
		}

		inline void setdofNames ( std::vector<std::string> dofNames )
		{
			_dofNames = dofNames;
		}

		inline void stopUpdate()
		{
			_dataTimer.stop();
		}

		inline void setPosition ( const std::vector<double>& position )
		{
			_openglWin->setPositionModel1 ( position );
		}

	protected:
		std::vector <double> _torqueGui;
		double _timeGui;
		std::vector<QCustomPlot*> _VectPlot;
		std::vector<QDockWidget*> _VectDock;
		std::vector<QTreeWidgetItem*> _VectTreeItem;
		std::vector<QPushButton*> _VectButtonItem;
		std::vector<QCPPlotTitle*> _VectTitle;
		std::vector<std::string> _channelNames;
		QTreeWidget* _treeViewRight;
		Ui::MainWindow* _ui;
		bool _firstPassTimeTorque;
		bool _firstPassTimeEmg;
		double _timeInitTorque;
		int _downscale;
		double _timeInitEmg;
		QWidget* _main;
		QSplitter* _splitter;
		QHBoxLayout* _mainHLayout;
		QVBoxLayout* _rightVLayout;
		QVBoxLayout* _layout;
		QScrollArea* _scrollArea;
		QScrollArea* _scrollAreaCheck;
		QWidget* _viewport;
		QWidget* _rightVLayoutWidget;
		GLWidget* _openglWin;
		std::vector<std::string> _dofNames;
		std::vector<bool> _checkBoxBool;
		QTimer _dataTimer;
		double _dt;
		std::string _modelFilename;
		unsigned int _nbOfPlot;
		unsigned int _cptWin;

		void closeEvent ( QCloseEvent* event );

	private slots:
		void _thread();
};

#endif /* GUI_H_ */
