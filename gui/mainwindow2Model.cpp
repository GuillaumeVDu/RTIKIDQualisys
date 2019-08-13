/*
 * Gui.cpp
 *
 *  Created on: May 29, 2015
 *      Author: gdurandau
 */

#include "mainwindow2Model.h"

using namespace glRT;

MainWindow::MainWindow ( double dt, std::vector<std::string> dofNames, const std::string modelFileName,
		QWidget* parent ) :
	_dofNames ( dofNames ), QMainWindow ( parent ), _ui ( new Ui::MainWindow ), _modelFilename ( modelFileName ), _dt ( dt )
{

	QRect rec = QApplication::desktop()->screenGeometry();

	resize ( rec.width(), rec.height() );

	_splitter = new QSplitter ( this );

	QSize sizeWin = size();

	_scrollAreaCheck = new QScrollArea ( _splitter );
	_scrollAreaCheck->setWidgetResizable ( true );
	_rightVLayoutWidget = new QWidget ( _scrollAreaCheck );
	_rightVLayout = new QVBoxLayout ( _rightVLayoutWidget );
	_scrollAreaCheck->setWidget ( _rightVLayoutWidget );
	_scrollAreaCheck->resize ( sizeWin.width() * 0.12,
			sizeWin.height() );

	QSize sizeHlay = _scrollAreaCheck->size();

	_scrollArea = new QScrollArea ( _splitter );
	_scrollArea->setWidgetResizable ( true );
	_viewport = new QWidget ( _scrollArea );
	_layout = new QVBoxLayout ( _viewport );
	_viewport->setMaximumWidth ( sizeWin.width() - sizeHlay.width() );
	_nbOfPlot = _dofNames.size();
	_viewport->setMinimumHeight ( _nbOfPlot * 300 );
	_scrollArea->setWidget ( _viewport );
	_scrollArea->resize ( sizeWin.width() * 0.6,
			sizeWin.height() );

	_openglWin = new GLWidget ( _modelFilename, _splitter, 2 );

	_openglWin->resize ( sizeWin.width() * 0.38,
			sizeWin.height() );


	_splitter->addWidget ( _scrollAreaCheck );
	_splitter->addWidget ( _scrollArea );
	_splitter->addWidget ( _openglWin );

	_scrollArea->resize ( sizeWin.width() - sizeHlay.width(), sizeHlay.height() );

	setCentralWidget ( _splitter );

	for ( std::vector<std::string>::const_iterator it = _dofNames.begin();
			it != _dofNames.end(); it++ )
	{
		_VectCheckBox.push_back (
			new QCheckBox ( QString ( it->c_str() ), _rightVLayoutWidget ) );
		_rightVLayout->addWidget ( _VectCheckBox.back() );
		_VectCheckBox.back()->setCheckState ( Qt::Unchecked );
	}

	_textError = new QTextEdit ( "Erreor: 0" );

	_rightVLayout->addWidget ( _textError );

	_rightVLayout->addStretch ( 1 );

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

	_firstPassTime = true;

	_openglWin->updateGL();
	_openglWin->ComputeRandPositionForModel2();

	connect ( &_dataTimer, SIGNAL ( timeout() ), this, SLOT ( _thread() ) );
	_dataTimer.start ( 50 ); // 30Hz
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

	for ( std::vector<QDockWidget*>::iterator it = _VectDock.begin();
			it != _VectDock.end(); it++ )
	{
		const int& cpt = std::distance<std::vector<QDockWidget*>::iterator> (
				_VectDock.begin(), it );

		if ( _VectCheckBox.at ( cpt )->checkState() == Qt::Unchecked
				&& _checkBoxBool.at ( cpt ) == true )
		{
			( *it )->hide();
			_nbOfPlot--;
			_checkBoxBool[cpt] = false;
		}
		else if ( _VectCheckBox.at ( cpt )->checkState() == Qt::Checked
				&& _checkBoxBool.at ( cpt ) == false )
		{
			( *it )->show();
			_nbOfPlot++;
			_checkBoxBool[cpt] = true;
		}
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

	SyncToolsIK::Shared::torqueMutex.lock();
	bool updateTorque = SyncToolsIK::Shared::newTorqueData;
	SyncToolsIK::Shared::torqueMutex.unlock();

	if ( updateTorque )
	{
		double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
		std::vector<double> torque;
		SyncToolsIK::Shared::torqueMutex.lock();
		torque = SyncToolsIK::Shared::torque;
		SyncToolsIK::Shared::torqueMutex.unlock();
		setTorque ( torque );

		if ( _torqueGui.size() != 0 )
		{
			//std::cout << _torqueGui.size() << std::endl;
			for ( std::vector<QCustomPlot*>::iterator it = _VectPlot.begin(); it != _VectPlot.end(); it++ )
			{
				const int& cpt = std::distance < std::vector<QCustomPlot*>::const_iterator > ( _VectPlot.begin(), it );

				if ( _checkBoxBool[cpt] )
				{
					( *it )->graph ( 0 )->addData ( key, _torqueGui.at ( cpt ) );
					( *it )->graph ( 0 )->rescaleValueAxis();
					( *it )->graph ( 0 )->removeDataBefore ( key - 7.50 );
					( *it )->xAxis->setRange ( key + 0.25, 8, Qt::AlignRight );
					( *it )->graph ( 0 )->rescaleValueAxis ( false, false );
					( *it )->replot();
				}
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
		SyncToolsIK::Shared::newMultTorqueData = false;
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
			Qtorque.resize ( _VectPlot.size() );
			QVector<double> Qtime;

			for ( std::vector<std::vector<double> >::const_iterator it = torque.begin(); it < torque.end(); it++ )
			{
				for ( std::vector<double>::const_iterator it2 = it->begin(); it2 < it->end(); it2++ )
				{
					const int& cpt = std::distance<std::vector<double>::const_iterator> ( it->begin(), it2 );
					Qtorque[cpt].push_back ( *it2 );
				}
			}

//  			std::cout << "torque: "<< Qtorque[0].size() << std::endl;

			if ( timeID.size() == 0 )
			{
				const int& cpt = Qtorque[0].size();
				double periode = ( _dt * NbOfComputation ) / cpt;

				for ( int i = 0; i < cpt; i++ )
				{
					Qtime.push_back ( key - ( ( cpt - i ) * periode ) );
				}
			}
			else
			{
				if ( _firstPassTime )
				{
					_firstPassTime = false;
					_timeInit = timeID[0];
				}

				for ( std::vector<double>::const_iterator itVect = timeID.begin(); itVect < timeID.end(); itVect++ )
					Qtime.push_back ( *itVect - _timeInit );
			}

//  			std::cout << "time: "<< Qtime.size() << " - " << Qtime[0] << " : " << Qtime.back() << std::endl;

			for ( std::vector<QCustomPlot*>::iterator it = _VectPlot.begin(); it != _VectPlot.end(); it++ )
			{
				const int& cpt = std::distance < std::vector<QCustomPlot*>::const_iterator > ( _VectPlot.begin(), it );

				if ( _checkBoxBool[cpt] )
				{
					( *it )->graph ( 0 )->addData ( Qtime, Qtorque.at ( cpt ) );
					( *it )->graph ( 0 )->rescaleValueAxis ( true );
					( *it )->graph ( 0 )->removeDataBefore ( Qtime.back() - 7.50 );
					( *it )->graph ( 0 )->rescaleValueAxis ( false, false );
					( *it )->xAxis->setRange ( Qtime.back() + 0.25, 8, Qt::AlignRight );
					( *it )->replot();
				}
			}
		}
	}

	double error = _openglWin->computeErrorBetweenModel();
// 	std::cout << "error: " << error << std::endl;
	std::stringstream textError;
	textError << "Error: " << error;
	_textError->setPlainText ( textError.str().c_str() );

	if ( error < 0.13 )
		_openglWin->ComputeRandPositionForModel2();
}

void MainWindow::closeEvent ( QCloseEvent* event )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
}

