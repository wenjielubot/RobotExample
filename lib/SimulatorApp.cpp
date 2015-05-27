#include "SimulatorApp.h"
#include <random>
#include <cmath>
#include <utility>
#include "time.h"
#include "utl.h"
#include <boost/thread/thread.hpp>
using namespace hll;

// SimulatorApp : Construcor / Discructor
SimulatorApp::SimulatorApp():ro_(*this), display(this), u(display) {
	
	// create a new thread
}
//SimulatorApp::~SimulatorApp() {}

// obstacle : Constructor
SimulatorApp::obstacle::obstacle(SimulatorApp& si):outer_(si)
{

    float buffer = 10;
    
    // x side length and y side length
    float xlen = fabs(outer_.boudingBox_[0].first - outer_.boudingBox_[1].first)/2-buffer;
    float ylen = fabs(outer_.boudingBox_[0].second - outer_.boudingBox_[1].second)/2-buffer;
	
	// center of bouding box
    float x0 = fabs(outer_.boudingBox_[0].first + outer_.boudingBox_[1].first)/2;
    float y0 = fabs(outer_.boudingBox_[0].second + outer_.boudingBox_[1].second)/2;

	// sample location_.first
    float xsample = sample(0,xlen);
    if (xsample >0)
        location_.first = constrain(xsample,xlen,buffer)+x0;
    if (xsample <0)
        location_.first = constrain(xsample, -buffer,-xlen)+x0;
	
	// sample location_.second
    float ysample = sample(0,ylen);
    if (ysample >0)
        location_.second = constrain(ysample,ylen,buffer)+y0;
    if (ysample <0)
        location_.second = constrain(ysample, -buffer,-ylen)+y0;
        
    // sample heading     
    theta_ = constrain(sample(0,6), M_PI,-M_PI);
}


// obstacle : static member 
float SimulatorApp::obstacle::vmax_ = 5 ;

// obstacle : setter
void SimulatorApp::obstacle::setVmax(float v)
{
    vmax_ = v;
}

/**
 * propagate obstacle state euler 
 * @param time step size
 */
void SimulatorApp::obstacle::propagate(float time)
{   
	// lock obstacle attributes
	boost::lock_guard<boost::mutex>  totalsLock(outer_.mutex);
	
	// update attibutes
    location_.first += cos(theta_)*speed_*time;
    location_.second += sin(theta_)*speed_*time;
	
	//check if new obstacle position is out of bouding box
    bool xbound = location_.first < outer_.boudingBox_[0].first  || location_.first > outer_.boudingBox_[1].first;
    bool ybound = location_.second> outer_.boudingBox_[0].second || location_.second< outer_.boudingBox_[1].second;
	
	// heading turn pi/2 if xbound is true and set position back
    float dth = 0;
    if (xbound) {
        location_.first -= cos(theta_)*speed_*time;
        dth += M_PI_2;
    }
    
    // heading turn pi/2 if xbound is true
    if (ybound) {
        location_.second -= sin(theta_)*speed_*time;
        dth += M_PI_2;
    }
	
	//update new obstacle position
    if (xbound || ybound )
    {
        theta_ += dth;
        location_.first += cos(theta_)*speed_*time;
        location_.second += sin(theta_)*speed_*time;
    }

	//add noise in obstacle speed and heading
    theta_ = sample(theta_,0.1);
    speed_ = sample(speed_,0.2);
}

/**
 * initialize robot state
 */
void SimulatorApp::robot::initialize()
{	
	// control range
    controlRange_ = M_PI/6;
    
    // set robot position at center of bouding box
    location_.first =  fabs(outer_.boudingBox_[0].first + outer_.boudingBox_[1].first)/2;
    location_.second = fabs(outer_.boudingBox_[0].second + outer_.boudingBox_[1].second)/2;
    
    //sample heading and speed
    theta_ = constrain(sample(0,6), M_PI,-M_PI);
    speed_ = constrain( fabs( sample(vmax_/2,3 )),vmax_,vmax_/2 );
}

// robot : Constructor
SimulatorApp::robot::robot(SimulatorApp& si):outer_(si){}

/**
 * propagate obstacle state euler 
 * @param time step size, control
 */
void SimulatorApp::robot::propagate(float time, float control)
{   
	// add lock on robot attributes
	boost::lock_guard<boost::mutex>  totalsLock(outer_.mutex);
	
	//update robot position and headings
    location_.first += cos(theta_)*speed_*time;
    location_.second += sin(theta_)*speed_*time;
    theta_ += speed_*tan(control)*time/10.0;
}

/**
 * initialize  SimulatorApp
 * @param obstacle number
 * @return simulator state
 */
SimulatorState SimulatorApp::initialize(int numObject) throw(std::runtime_error)
{
    try
    {	
		// if SimulatorApp has been initialized
		if (boudingBox_.size()>1)
			throw std::runtime_error("simulator has been initialized");
		
		// set boudingBox_
        boudingBox_.resize(2);
        boudingBox_[0].first = -100;
        boudingBox_[0].second = 100;
        boudingBox_[1].first = 100;
        boudingBox_[1].second = -100;
		
		// add obstacles
        for (int i=0; i<numObject; ++i)
        {
            obstacle tmp(*this);
            ob_.insert(std::pair<int64_t,obstacle>(i,tmp));
        }
		
		// initialize robot
        ro_.initialize();
		
		//update currentState_
        updateState();
        
		//update time
        currentTime_ = time(0);
    } catch(std::runtime_error& e)
    {
        throw (e);
    }

    return getState();
}

/**
 * check if robot is outside of obstacles
 */
bool SimulatorApp::validityChecker()
{
    pointD2 pos = ro_.location_;

    if (!inBox()) return false;
    
    // check if robot is in obstacles
    for (auto it=ob_.begin(); it!=ob_.end(); ++it)
    {

        float dx = it->second.location_.first-pos.first;
        float dy = it->second.location_.second-pos.second;

        if (sqrt(pow(dx,2)+pow(dy,2))<10)
            return false;
    }

    return true;
}

/**
 * check if object is in bouding box
 */
bool SimulatorApp::inBox()
{
    pointD2 location = ro_.location_;

    bool xbound = location.first < boudingBox_[0].first  || location.first > boudingBox_[1].first;
    bool ybound = location.second> boudingBox_[0].second || location.second< boudingBox_[1].second;
    return !(xbound || ybound);

}

/**
 * advance SimulatorApp, called by initialize ()  from Simulator
 * @param control, time elapsed
 * @return simulation state of advancing dtime based on previous control,
 * 		   current control is recored and will be used next time advance() is called
 */
SimulatorState SimulatorApp::advance(double steering_angle, double elapsed_time) throw(std::range_error, std::runtime_error)
{
	// throw range error if input control is too large
    if (fabs(steering_angle) > fabs(ro_.controlRange_))
        throw std::range_error("Some error in the range!");

	
    float t = 0;
    
    // propagete robot and obstacle state
    // throw runtime_error if hit
    while (t<elapsed_time)
    {
        t += timeStep_;
        for (auto it=ob_.begin(); it!=ob_.end(); ++it)
        {
            it->second.propagate(timeStep_);
        }

        ro_.propagate(timeStep_,steering_angle);

        if(!validityChecker())
        {
            updateState();
            throw std::runtime_error("Hit");
        }
    }

    updateState();
    return getState();
}

// SimulatorApp getter
SimulatorState SimulatorApp::getState() const {
    return currentState_;
}

/**
 * update currentState_ based on ro_ and ob_ 
 */
void  SimulatorApp::updateState()
{
    currentState_.robotState.location = ro_.location_;
    currentState_.robotState.theta = ro_.theta_;

    for (auto it=ob_.begin(); it!=ob_.end(); ++it)
    {
        currentState_.obstacleStates[ it->first ] = it->second.location_;
    }
    currentState_.runTime = currentTime_;
}

// SimulatorApp getter and setter 
time_t  SimulatorApp::getTime() const 
{
    return currentTime_;
}

void SimulatorApp::setTime(time_t t )
{
    currentTime_ = t;
}

pointsD2 SimulatorApp::getBound() const
{
    return boudingBox_;
}

// DisplayD2 : Constructor
DisplayD2::DisplayD2(Graphic* p,QWidget *parent)
    : QWidget(parent),ptr(p)
{
    setWindowTitle(tr("Display"));
    resize(500, 500);
}

/**
 * paint event , called by Qwidget update() when timeout() is signaled 
 */
void DisplayD2::paintEvent(QPaintEvent *)
{
	
	// color
    QColor hourColor(127, 0, 0);
    QColor minuteColor(0, 127, 0);

    int side = qMin(width(), height());
	
	// create painter
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(width() / 2, height() / 2);
    painter.scale(side / 200.0, side / 200.0);

    painter.setPen(Qt::NoPen);
    painter.setBrush(minuteColor);
    painter.save();



    {
		// create lock on ob_ and ro_
        boost::lock_guard<boost::mutex>  totalsLock(ptr->parentPtr->mutex);
        // copy data from ob_ and ro_ to ob and ro
        ob.resize(ptr->parentPtr->ob_.size());
        int i=0;
        for (auto it=ptr->parentPtr->ob_.begin(); it!=ptr->parentPtr->ob_.end(); ++it)
        {
            ob[i].first = it-> second.location_.first;
            ob[i].second = it-> second.location_.second;
            ++i;
        }
        ro.first = ptr->parentPtr->ro_.location_.first;
        ro.second = ptr->parentPtr->ro_.location_.second;
    } // release lock
	
	// draw obstacles
    for (auto it=ob.begin(); it!=ob.end(); ++it)
    {
        paintDisk(*it,painter);
    }
    
    painter.restore();
    painter.setBrush(hourColor);
	
	//draw robot
    paintDisk(ro,painter);
}

/**
 * draw disk 
 * @param position
 * @param painter
 */
void DisplayD2::paintDisk(pointD2 pos,QPainter& pter)
{
    QPoint center(pos.first,pos.second);
    pter.drawEllipse(center,5,5);
}



