

#ifndef SimulatorAPP_H_
#define SimulatorAPP_H_

#include "Simulator.h"
#include <vector>
#include <time.h>
#include <QWidget>
#include <QApplication>
#include <iostream>
#include <boost/thread.hpp>
#include <QtWidgets>
//#include <boost/thread/mutex.hpp>
#include <QObject>

namespace hll
{
	//forward declearation
	class Graphic;
	
	/**
	 * DisplayD2 for display robot and obstacle positions
	 */
	class DisplayD2 : public QWidget
	{
			//Qwidget required
			Q_OBJECT
	
	    public:
			//
			// Constructor
	        DisplayD2(Graphic* p,QWidget *parent = 0);
	        
	        //
	        // Methods
	        void paintDisk(pointD2 pos, QPainter& pter);
	
	    protected:
	    
			// Other methods
	        void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
	
	    public:
			
			//
			// Attributes 
	        Graphic* ptr;
	        std::vector<pointD2> ob;
	        pointD2 ro;
	};
	
	// Forward declearation
	class SimulatorApp;
	
	/**
	 * Graphic class for display DisplayD2
	 */
	class  Graphic
	{
	    public:
	    
			// Constructor
			
	        Graphic(SimulatorApp* p): parentPtr(p) {}
	        virtual ~Graphic()
	        {
				//displayd2->close(); 
				//displayd2->disconnect(); 
				//delete displayd2; 
				//delete timer; 
				//app->exit();
				//delete app;
			}
			// Functor used by Boost::thread
	        void operator()()
	        {
				// Dummy parameters for creating Qaaplication
	            int staticArgc = 1;
	            char staticDummy[] = "QtLibApplication";
	            char *staticArgv[2] = { staticDummy, nullptr };
				
				// Create QApplication 
	            app = new QApplication(staticArgc,staticArgv);
	            
	            // Create QWidget link to Graphic
	            displayd2 = new DisplayD2(this);
	            
	            // Create timer for updating graphic
	            timer = new QTimer(displayd2);
	            
	            // Connect Qwidget update() with timeout() signal from timer
	            displayd2->connect(timer, SIGNAL(timeout()), displayd2, SLOT(update()));
	            timer->start(50);
	            
	            displayd2->show();
	            
	            //requred by Qwidgets
	            app->exec();
	        }
	
	        void update();
	
			//
			//Attributes
			
	        SimulatorApp* parentPtr; //pointer to SimulatorApp
	        DisplayD2* displayd2; //pointer to widget
	        QTimer* timer;
	        QApplication *app;
	};
	
	
	/**
	 * simulator application, instantiated by Simulator 
	 */	
	class SimulatorApp {
	
	    public:
			
			/**
			 * struct for each obstacle
			 */
	        struct obstacle 
	        {
				// Constructor 
	            
	            obstacle (SimulatorApp& si);
				
				// Methods	
				
	            void propagate(float time);
	
	            static void setVmax(float v);
	            
	            // same maximum speed for all obstacles
	            static float vmax_;
	            
	            // own attributes
	            
	            pointD2 location_;
	            float theta_;
	            float speed_;
	            
	            // reference to outer class SimulatorApp
	            SimulatorApp& outer_;
	        };

			/**
			 * struct for each robot
			 */	
	        struct robot
	        {
				// Constructor
	            robot(SimulatorApp& si);
	            
	            // Methods
	            void initialize();
	            void propagate(float time, float control);
	            
	            // Attributes
	            
	            pointD2 location_;
	            float theta_;
	            float speed_;
	            float vmax_=5;
	            float controlRange_;
	            
	            // reference to outer class SimulatorApp
	            SimulatorApp& outer_;
	        };
	        			
			//
			// Constructor / Distructor
			
	        SimulatorApp();
	        virtual ~SimulatorApp(){u.interrupt();u.join();}
	        
	        // Methods
	        
	        /**
	         * initialize SimulatorApp, called by initialize ()  from Simulator
	         * @param number of obstacles
	         * @return initial simulation state
	         */
	        SimulatorState initialize(int numObject) throw(std::runtime_error);
	        
	        /**
	         * advance SimulatorApp, called by initialize ()  from Simulator
	         * @param control, time elapsed
	         * @return simulation state of advancing dtime based on previous control,
	         * 		   current control is recored and will be used next time advance() is called
	         */
	        SimulatorState advance(double control, double dtime) throw(std::range_error, std::runtime_error);
	
			/**
			 * update currentState_ based on ro_ and ob_ 
			 */
			void updateState();
	
			/**
			 * check if robot is outside of obstacles , check if object is in bouding box
			 */
	        bool validityChecker();
	        bool inBox();
			
			// Attribute accessor	
	        
	        pointsD2 getBound() const;
	
	        boost::mutex& getMutex() { return mutex;}
	
			time_t getTime() const;
	        
	        void setTime(time_t t );
	
	        SimulatorState getState() const;
	        
	        
			// Attributes
			
			//obstacle map and robot
	        std::map<int64_t, obstacle> ob_;
	        robot ro_;
	        
	        //simulation state
	        SimulatorState currentState_;
	        time_t currentTime_;
	        pointsD2 boudingBox_;
	        
	        // time step in simulation
	        float timeStep_ = 0.2;
			
			// Garphic instance for display
	        Graphic display;
			
			// mutual exception for thread operations
	        boost::mutex mutex;	
	        
	        boost::thread u;
	};

}
#endif /* SimulatorApp_H_ */
