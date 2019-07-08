/*
 * Gui.cpp
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#include "mainwindowEMG.h"
using namespace gl;

MainWindow::MainWindow ( double dt, std::vector<std::string> dofNames, const std::string modelFileName, std::vector<std::string> channelNames,
		QWidget* parent ) :
	_dofNames ( dofNames ), QMainWindow ( parent ), _ui ( new Ui::MainWindow ), _modelFilename ( modelFileName ), _dt ( dt ), _channelNames ( channelNames )
{

	QRect rec = QApplication::desktop()->screenGeometry();

	resize ( rec.width(), rec.height() );

	_splitter = new QSplitter ( this );

	QSize sizeWin = size();

	_scrollAreaCheck = new QScrollArea ( _splitter );
	_splitter->addWidget ( _scrollAreaCheck );
	_scrollAreaCheck->setWidgetResizable ( true );
	_treeViewRight = new QTreeWidget ( _scrollAreaCheck );
	_treeViewRight->setHeaderLabels (
		QStringList() << "Name" << "save" );
	_treeViewRight->header()->close();
	_treeViewRight->header()->resizeSection ( 0, 180 );
	_treeViewRight->header()->resizeSection ( 1, 10 );
	_treeViewRight->header()->setStretchLastSection ( false );
	_scrollAreaCheck->setWidget ( _treeViewRight );
	_scrollAreaCheck->resize ( sizeWin.width() * 0.1,
			sizeWin.height() );

	QSize sizeHlay = _scrollAreaCheck->size();

	_scrollArea = new QScrollArea ( _splitter );
	_scrollArea->setWidgetResizable ( true );
	_viewport = new QWidget ( _scrollArea );
	_layout = new QVBoxLayout ( _viewport );
	_viewport->setMaximumWidth ( sizeWin.width() - sizeHlay.width() );
	_nbOfPlot = _dofNames.size() + _channelNames.size();
	_viewport->setMinimumHeight ( _nbOfPlot * 300 );
	_scrollArea->setWidget ( _viewport );
	_scrollArea->resize ( sizeWin.width() * 0.6,
			sizeWin.height() );

//
//	_openglWin = new GLWidget(_modelFilename, _splitter, 2);
	_openglWin = new GLWidget ( _modelFilename, _splitter );

//	_openglWin->createModel1();

	_openglWin->resize ( sizeWin.width() * 0.38,
			sizeWin.height() );


	_splitter->addWidget ( _scrollAreaCheck );
	_splitter->addWidget ( _scrollArea );
	_splitter->addWidget ( _openglWin );

	_scrollArea->resize ( sizeWin.width() - sizeHlay.width(), sizeHlay.height() );

	setCentralWidget ( _splitter );

	unsigned int cpt = 0;

	_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeViewRight ) );
	_VectTreeItem.back()->setText ( 0, "EMG" );

	for ( std::vector<std::string>::const_iterator it = _channelNames.begin();
			it != _channelNames.end(); it++ )
	{
		_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem.at ( cpt ) ) );
		_VectButtonItem.push_back ( new QPushButton ( "Save", _treeViewRight ) );
		_VectTreeItem.back()->setText ( 0, QString ( ( *it ).c_str() ) );
		_VectTreeItem.back()->setData ( 0, Qt::CheckStateRole, Qt::Unchecked );
		_treeViewRight->setItemWidget ( _VectTreeItem.back(), 1, _VectButtonItem.back() );
	}

	cpt = _channelNames.size() + 1;

	_VectTreeItem.push_back ( new QTreeWidgetItem ( _treeViewRight ) );
	_VectTreeItem.back()->setText ( 0, "Torque" );

	for ( std::vector<std::string>::const_iterator it = _dofNames.begin();
			it != _dofNames.end(); it++ )
	{
		_VectTreeItem.push_back ( new QTreeWidgetItem ( _VectTreeItem.at ( cpt ) ) );
		_VectButtonItem.push_back ( new QPushButton ( "Save", _treeViewRight ) );
		_VectTreeItem.back()->setText ( 0, QString ( ( *it ).c_str() ) );
		_VectTreeItem.back()->setData ( 0, Qt::CheckStateRole, Qt::Unchecked );
		_treeViewRight->setItemWidget ( _VectTreeItem.back(), 1, _VectButtonItem.back() );
	}

	for ( std::vector<std::string>::const_iterator it = _channelNames.begin();
			it != _channelNames.end(); it++ )
	{
		_VectDock.push_back ( new QDockWidget ( this ) );
		_VectPlot.push_back ( new QCustomPlot ( _VectDock.back() ) );
		_VectTitle.push_back ( new QCPPlotTitle ( _VectPlot.back() ) );
		_layout->addWidget ( _VectDock.back() );
		std::stringstream emgText;
		emgText << "EMG " + *it;
		_VectTitle.back()->setText ( emgText.str().c_str() );
		_VectTitle.back()->setFont ( QFont ( "sans", 12, QFont::Bold ) );
		_VectTitle.back()->setTextColor ( Qt::blue );
		_VectPlot.back()->plotLayout()->insertRow ( 0 );
		_VectPlot.back()->plotLayout()->addElement ( 0, 0, _VectTitle.back() );
		_VectPlot.back()->addGraph(); // blue line
		QPen bluePen;
		bluePen.setColor ( Qt::blue );
		bluePen.setWidthF ( 3 );
		_VectPlot.back()->graph ( 0 )->setPen ( bluePen );
		_VectPlot.back()->graph ( 0 )->setBrush ( QBrush ( QColor ( 240, 255, 200 ) ) );
		_VectPlot.back()->graph ( 0 )->setAntialiasedFill ( false );

		_VectPlot.back()->xAxis->setTickLabelType ( QCPAxis::ltDateTime );
		_VectPlot.back()->xAxis->setDateTimeFormat ( "hh:mm:ss" );
		_VectPlot.back()->xAxis->setAutoTickStep ( false );
		_VectPlot.back()->xAxis->setTickStep ( 2 );
		_VectPlot.back()->axisRect()->setupFullAxesBox();

		// make left and bottom axes transfer their ranges to right and top axes:
		connect ( _VectPlot.back()->xAxis, SIGNAL ( rangeChanged ( QCPRange ) ),
				_VectPlot.back()->xAxis2, SLOT ( setRange ( QCPRange ) ) );
		connect ( _VectPlot.back()->yAxis, SIGNAL ( rangeChanged ( QCPRange ) ),
				_VectPlot.back()->yAxis2, SLOT ( setRange ( QCPRange ) ) );
		_VectPlot.back()->yAxis->setLabel ( "Normalize" );
		_VectPlot.back()->xAxis->setLabel ( "Time" );
		_VectDock.back()->setWidget ( _VectPlot.back() );
		_VectDock.back()->hide();
		_checkBoxBool.push_back ( false );
		_nbOfPlot--;
	}

	for ( std::vector<std::string>::const_iterator it = _dofNames.begin();
			it != _dofNames.end(); it++ )
	{
		_VectDock.push_back ( new QDockWidget ( _viewport ) );
		_VectPlot.push_back ( new QCustomPlot ( _VectDock.back() ) );
		_VectTitle.push_back ( new QCPPlotTitle ( _VectPlot.back() ) );
		_layout->addWidget ( _VectDock.back() );
		std::stringstream torqueText;
		torqueText << "Torque " + *it;
		_VectTitle.back()->setText ( torqueText.str().c_str() );
		_VectTitle.back()->setFont ( QFont ( "sans", 12, QFont::Bold ) );
		_VectTitle.back()->setTextColor ( Qt::darkGreen );
		_VectPlot.back()->plotLayout()->insertRow ( 0 );
		_VectPlot.back()->plotLayout()->addElement ( 0, 0, _VectTitle.back() );
		_VectPlot.back()->addGraph(); // blue line
		QPen pen;
		pen.setColor ( Qt::blue );
		pen.setWidthF ( 3 );
		_VectPlot.back()->graph ( 0 )->setPen ( pen );
		_VectPlot.back()->graph ( 0 )->setAntialiasedFill ( true );

		_VectPlot.back()->xAxis->setTickLabelType ( QCPAxis::ltDateTime );
		_VectPlot.back()->xAxis->setDateTimeFormat ( "hh:mm:ss" );
		_VectPlot.back()->xAxis->setAutoTickStep ( false );
		_VectPlot.back()->xAxis->setTickStep ( 2 );
		_VectPlot.back()->axisRect()->setupFullAxesBox();

		// make left and bottom axes transfer their ranges to right and top axes:
		connect ( _VectPlot.back()->xAxis, SIGNAL ( rangeChanged ( QCPRange ) ),
				_VectPlot.back()->xAxis2, SLOT ( setRange ( QCPRange ) ) );
		connect ( _VectPlot.back()->yAxis, SIGNAL ( rangeChanged ( QCPRange ) ),
				_VectPlot.back()->yAxis2, SLOT ( setRange ( QCPRange ) ) );
		_VectDock.back()->setWidget ( _VectPlot.back() );
		_checkBoxBool.push_back ( false );
		_VectDock.back()->hide();
		_nbOfPlot--;
	}

	_firstPassTimeTorque = _firstPassTimeEmg = true;
	_timeInitTorque = _timeInitEmg = 0;


	_openglWin->updateGL();
//	_openglWin->ComputeRandPositionForModel2();

	connect ( &_dataTimer, SIGNAL ( timeout() ), this, SLOT ( _thread() ) );
	_dataTimer.start ( 33 ); // 30Hz
}

MainWindow::~MainWindow()
{
	delete _ui;
}

void MainWindow::_thread()
{
	//boost::timer::auto_cpu_timer auto_t3;
	SyncToolsIK::Shared::endThreadMutex.lock();

	if ( SyncToolsIK::Shared::endThread )
	{
		SyncToolsIK::Shared::endThreadMutex.unlock();
		stopUpdate();
	}

	SyncToolsIK::Shared::endThreadMutex.unlock();

	unsigned int cpt = 1;
	unsigned int cptDock = 0;

	for ( std::vector<std::string>::const_iterator it = _channelNames.begin(); it != _channelNames.end(); it++ )
	{
		if ( _VectTreeItem.at ( cpt )->checkState ( 0 ) == Qt::Unchecked
				&& _checkBoxBool.at ( cptDock ) == true )
		{
			_VectDock.at ( cptDock )->hide();
			_nbOfPlot--;
			_checkBoxBool[cptDock] = false;
		}
		else if ( _VectTreeItem.at ( cpt )->checkState ( 0 ) == Qt::Checked
				&& _checkBoxBool.at ( cptDock ) == false )
		{
			_VectDock.at ( cptDock )->show();
			_nbOfPlot++;
			_checkBoxBool[cptDock] = true;
		}

		cpt++;
		cptDock++;
	}

	cpt++;

	for ( std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++ )
	{
		if ( _VectTreeItem.at ( cpt )->checkState ( 0 ) == Qt::Unchecked
				&& _checkBoxBool.at ( cptDock ) == true )
		{
			_VectDock.at ( cptDock )->hide();
			_nbOfPlot--;
			_checkBoxBool[cptDock] = false;
		}
		else if ( _VectTreeItem.at ( cpt )->checkState ( 0 ) == Qt::Checked
				&& _checkBoxBool.at ( cptDock ) == false )
		{
			_VectDock.at ( cptDock )->show();
			_nbOfPlot++;
			_checkBoxBool[cptDock] = true;
		}

		cpt++;
		cptDock++;
	}

	cpt = 0;

	for ( std::vector<std::string>::const_iterator it = _channelNames.begin(); it != _channelNames.end(); it++ )
	{
		if ( _VectButtonItem.at ( cpt )->isDown() == true )
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "EMG_" << *it << "_" << date.toStdString() << ".pdf";
			_VectPlot.at ( cpt )->savePdf ( QString ( text.str().c_str() ) );
		}

		cpt++;
	}

	for ( std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++ )
	{
		if ( _VectButtonItem.at ( cpt )->isDown() == true )
		{
			QString date = QDateTime::currentDateTime().toString();
			std::stringstream text;
			text << "Torque_" << *it << "_" << date.toStdString() << ".pdf";
			_VectPlot.at ( cpt )->savePdf ( QString ( text.str().c_str() ) );
		}

		cpt++;
	}

	QSize sizeWin = size();
	_viewport->setMinimumHeight ( _nbOfPlot * 300 );

	SyncToolsIK::Shared::positionMutex.lock();
	bool updateik = SyncToolsIK::Shared::newPositionData;
	SyncToolsIK::Shared::positionMutex.unlock();

	if ( updateik )
	{
		std::vector<double> position;
		SyncToolsIK::Shared::positionMutex.lock();
		position = SyncToolsIK::Shared::position;
		SyncToolsIK::Shared::newPositionData = false;
		SyncToolsIK::Shared::positionMutex.unlock();
		setPosition ( position );
		_openglWin->updateGL();
	}

	SyncToolsEMG::Shared::EMGMutex.lock();
	bool updateEmg = SyncToolsEMG::Shared::newEmg;
	SyncToolsEMG::Shared::EMGMutex.unlock();

	cpt = 0;

	if ( updateEmg )
	{
		SyncToolsEMG::Shared::EMGMutex.lock();
		std::vector<std::vector<double> > emg = SyncToolsEMG::Shared::emg;
		std::vector<double> timeEMG = SyncToolsEMG::Shared::timeEMG;
		SyncToolsEMG::Shared::newEmg = false;
		SyncToolsEMG::Shared::EMGMutex.unlock();

		if ( emg.size() != 0 && timeEMG.size() != 0 )
		{
			QVector<QVector<double> > QEmg;
			QEmg.resize ( _channelNames.size() );
			QVector<double> Qtime;
			
			if ( _firstPassTimeEmg )
			{
				if ( timeEMG[0] - timeEMG[1] < 0.001 )
					_downscale = 20;
				else if ( timeEMG[0] - timeEMG[1] < 0.01 )
					_downscale = 10;
				else
					_downscale = 1;

				_firstPassTimeEmg = false;
				_timeInitEmg = timeEMG[0];
			}

			for ( std::vector<std::vector<double> >::const_iterator it = emg.begin(); it < emg.end(); it = it + _downscale )
			{
// 				const int& cpt1 = std::distance<std::vector<std::vector<double> >::const_iterator> ( emg.begin(), it );
				for ( std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++ )
				{
					const int& cpt2 = std::distance<std::vector<double>::const_iterator> ( it->begin(), it2 );
					QEmg[cpt2].push_back ( *it2 );
				}
			}



			for ( std::vector<double>::const_iterator itVect = timeEMG.begin(); itVect < timeEMG.end(); itVect = itVect + _downscale )
				Qtime.push_back ( *itVect - _timeInitEmg );

			for ( std::vector<std::string>::const_iterator it = _channelNames.begin(); it != _channelNames.end(); it++ )
			{
				if ( _checkBoxBool[cpt] )
				{
					const int& i = std::distance < std::vector<std::string>::const_iterator > ( _channelNames.begin(), it );
					_VectPlot.at ( cpt )->graph ( 0 )->addData ( Qtime, QEmg.at ( i ) );
					_VectPlot.at ( cpt )->xAxis->setRange ( Qtime.back() + 0.25, 8, Qt::AlignRight );
					_VectPlot.at ( cpt )->graph ( 0 )->removeDataBefore ( Qtime.back() - 7.50 );
					_VectPlot.at ( cpt )->yAxis->setRange ( 1, 0 );
					_VectPlot.at ( cpt )->replot();
				}

				cpt++;
			}
		}
	}

	SyncToolsIK::Shared::torqueMutex.lock();
	bool updateTorque = SyncToolsIK::Shared::newTorqueData;
	SyncToolsIK::Shared::torqueMutex.unlock();

	cpt = _channelNames.size();

	if ( updateTorque )
	{
		double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
		std::vector<double> torque;
		SyncToolsIK::Shared::torqueMutex.lock();
		torque = SyncToolsIK::Shared::torque;
		SyncToolsIK::Shared::newTorqueData = false;
		SyncToolsIK::Shared::torqueMutex.unlock();
		setTorque ( torque );

		if ( _torqueGui.size() != 0 )
		{
			for ( std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++ )
			{
				if ( _checkBoxBool[cpt] )
				{
					const int& i = std::distance < std::vector<std::string>::const_iterator > ( _dofNames.begin(), it );
					_VectPlot.at ( cpt )->graph ( 0 )->addData ( key, _torqueGui.at ( i ) );
					_VectPlot.at ( cpt )->graph ( 0 )->rescaleValueAxis();
					_VectPlot.at ( cpt )->graph ( 0 )->removeDataBefore ( key - 7.50 );
					_VectPlot.at ( cpt )->xAxis->setRange ( key + 0.25, 8, Qt::AlignRight );
					_VectPlot.at ( cpt )->graph ( 0 )->rescaleValueAxis ( false, false );
					_VectPlot.at ( cpt )->replot();
				}

				cpt++;
			}
		}
	}

	SyncToolsIK::Shared::MultTorqueMutex.lock();
	updateTorque = SyncToolsIK::Shared::newMultTorqueData;
// 	std::cout << updateTorque << std::endl;
	SyncToolsIK::Shared::MultTorqueMutex.unlock();

	if ( updateTorque )
	{
		double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
		std::vector<std::vector<double> > torque;
		SyncToolsIK::Shared::MultTorqueMutex.lock();
		torque = SyncToolsIK::Shared::MultTorque;
		SyncToolsIK::Shared::newTorqueData = false;
		SyncToolsIK::Shared::MultTorque.clear();
		std::vector<double> timeID = SyncToolsIK::Shared::timeTorque;
		SyncToolsIK::Shared::timeTorque.clear();
		unsigned int NbOfComputation = SyncToolsIK::Shared::NbOfComputation;
		SyncToolsIK::Shared::NbOfComputation = 0;
		SyncToolsIK::Shared::MultTorqueMutex.unlock();

//  		std::cout << torque.size() << std::endl;

		//setTorque(torque);
		if ( torque.size() != 0 )
		{
//			std::cout << torque.back().size() << std::endl;
			QVector<QVector<double> > Qtorque;
			Qtorque.resize ( _dofNames.size() );
			QVector<double> Qtime;

			for ( std::vector<std::vector<double> >::const_iterator it = torque.begin(); it < torque.end(); it++ )
			{
				for ( std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++ )
				{
					const int& cpt2 = std::distance<std::vector<double>::const_iterator> ( it->begin(), it2 );
					Qtorque[cpt2].push_back ( *it2 );
				}
			}

//  			std::cout << "torque: "<< Qtorque[0].size() << std::endl;

			if ( timeID.size() == 0 )
			{
				const int& cpt1 = Qtorque[0].size();
				double periode = ( _dt * NbOfComputation ) / cpt1;

				for ( int i = 0; i < cpt1; i++ )
				{
					Qtime.push_back ( key - ( ( cpt1 - i ) * periode ) );
				}
			}
			else
			{
				if ( _firstPassTimeTorque )
				{
					_firstPassTimeTorque = false;
					_timeInitTorque = timeID[0];
				}

				for ( std::vector<double>::const_iterator itVect = timeID.begin(); itVect < timeID.end(); itVect++ )
					Qtime.push_back ( *itVect - _timeInitTorque );
			}

//  			std::cout << "time: "<< Qtime.size() << " - " << Qtime[0] << " : " << Qtime.back() << std::endl;

			for ( std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++ )
			{
				if ( _checkBoxBool[cpt] )
				{
					const int& i = std::distance < std::vector<std::string>::const_iterator > ( _dofNames.begin(), it );
					_VectPlot.at ( cpt )->graph ( 0 )->addData ( Qtime, Qtorque.at ( i ) );
					_VectPlot.at ( cpt )->graph ( 0 )->rescaleValueAxis ( true );
					_VectPlot.at ( cpt )->graph ( 0 )->removeDataBefore ( Qtime.back() - 7.50 );
					_VectPlot.at ( cpt )->graph ( 0 )->rescaleValueAxis ( false, false );
					_VectPlot.at ( cpt )->xAxis->setRange ( Qtime.back() + 0.25, 8, Qt::AlignRight );
					_VectPlot.at ( cpt )->replot();
				}

				cpt++;
			}
		}
	}

//	double error = _openglWin->computeErrorBetweenModel();
////	std::cout << "error: " << error << std::endl;
//	if(error < 0.13)
//		_openglWin->ComputeRandPositionForModel2();
}

void MainWindow::closeEvent ( QCloseEvent* event )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
}
