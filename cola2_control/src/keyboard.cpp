
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/*
 * keyboard.cpp
 *
 *  Created on: 17/04/2013
 *      Author: Eduard
 */

/*@@>This node is used to drive the AUV from a linux terminal using keyboard commands.<@@*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <termios.h>
#include <stdio.h>
#include <signal.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


// Flag used to stop the node
sig_atomic_t volatile keyboard_requested_shutdown = 0 ;


class Keyboard{
private:
    // Node handle
    ros::NodeHandle n ;

    // Number of buttons
    int NBUTTONS ;

    // Read keybard thread
    boost::mutex _setpoints_mutex ;
    boost::shared_ptr<boost::thread> reading_thread ;
    bool shutdownThread ;

    // Publisher
    ros::Publisher pub ;

    // Subscriber
    ros::Subscriber sub_ok ;

public:
    Keyboard() { }
    ~Keyboard() { }

    // Terminal struct
    struct termios oldt, newt ;

    void init() {
        // Number of buttons
        NBUTTONS = 19 ;

        // Publishers
        
        pub = n.advertise<sensor_msgs::Joy>( "/keyboard", 1 ) ;

        // Init thread
        shutdownThread = false ;
        reading_thread = boost::shared_ptr<boost::thread>( new boost::thread( &Keyboard::readKeyboardHits, this ) ) ;

        ROS_INFO( "/keyboard: initialized" ) ;
    }


    int getChar() {
        int ch ;
        tcgetattr( STDIN_FILENO, &oldt ) ;
        memcpy(&newt, &oldt, sizeof(struct termios));
        newt.c_lflag &= ~( ICANON | ECHO ) ;
        newt.c_cc[VEOL] = 1;
        newt.c_cc[VEOF] = 2;
        tcsetattr( STDIN_FILENO, TCSANOW, &newt ) ;
        ch = getchar() ;
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt ) ;
        return ch ;
    }


    void readKeyboardHits() {
        // Initialize buttons
        std::vector<int> buttons ;
        for( int iter = 0 ; iter != NBUTTONS ; iter++ ) {
            buttons.push_back( 0 ) ;
        }

        // Forward, backward, turn left and turn right
        const int KEY_W = 87 ;
        const int KEY_w = 119 ;
        const int KEY_S = 83 ;
        const int KEY_s = 115 ;
        const int KEY_A = 65 ;
        const int KEY_a = 97 ;
        const int KEY_D = 68 ;
        const int KEY_d = 100 ;

        // For yaw and heave velocities
        const int KEY_T = 84 ;
        const int KEY_t = 116 ;
        const int KEY_G = 71 ;
        const int KEY_g = 103 ;
        const int KEY_F = 70 ;
        const int KEY_f = 102 ;
        const int KEY_H = 72 ;
        const int KEY_h = 104 ;

        // Up and down
        const int KEY_UP = 65 ;
        const int KEY_DOWN = 66 ;
        const int KEY_RIGHT = 67 ;
        const int KEY_LEFT = 68 ;

        // Pitch
        const int KEY_O = 79 ;
        const int KEY_o = 111 ;
        const int KEY_K = 75 ;
        const int KEY_k = 107 ;
        
        // Pitch mode
        const int KEY_M = 77 ;
        const int KEY_m = 109 ;
        const int KEY_N = 78 ;
        const int KEY_n = 110 ;

        // Esc and Space control actions
        const int KEY_ESC = 27 ;
        const int KEY_OBRACKET = 91 ;
        const int KEY_SPACE = 32 ;
        const int KEY_DOT = 46 ;
        const int KEY_COMA = 44 ;

        while( !keyboard_requested_shutdown )
        {
            // Set vector to non-pressed
            for( int iter = 0 ; iter != NBUTTONS ; iter++ ) {
                buttons[iter] = 0 ;
            }

            // Find which button is pressed
            int key = getChar() ;
            // std::cout << key << std::endl;

            if ( key == KEY_SPACE ) {
                buttons[0] = 1 ;
            }
            else if ( key == KEY_W || key == KEY_w ) {
                buttons[1] = 1 ;
            }
            else if ( key == KEY_S || key == KEY_s ) {
                buttons[2] = 1 ;
            }
            else if ( key == KEY_A || key == KEY_a ) {
                buttons[3] = 1 ;
            }
            else if ( key == KEY_D || key == KEY_d ) {
                buttons[4] = 1 ;
            }
            else if ( key == KEY_ESC ) {
                if ( getChar() == KEY_OBRACKET ) {
                    int arrow_key = getChar() ;

                    if ( arrow_key == KEY_UP ) {
                        buttons[5] = 1 ;
                    }
                    else if ( arrow_key == KEY_DOWN ) {
                        buttons[6] = 1 ;
                    }
                    else if ( arrow_key == KEY_RIGHT ) {
                        buttons[7] = 1 ;
                    }
                    else if ( arrow_key == KEY_LEFT ) {
                        buttons[8] = 1 ;
                    }
                }
            }
            else if ( key == KEY_O || key == KEY_o ) {
                buttons[9] = 1 ;
            }
            else if ( key == KEY_K || key == KEY_k ) {
                buttons[10] = 1 ;
            }
            else if ( key == KEY_DOT ) {
                buttons[11] = 1 ;
            }
            else if ( key == KEY_COMA ) {
                buttons[12] = 1 ;
            }
            else if ( key == KEY_T || key == KEY_t ) {
                buttons[13] = 1 ;
            }
            else if ( key == KEY_G || key == KEY_g ) {
                buttons[14] = 1 ;
            }
            else if ( key == KEY_F || key == KEY_f ) {
                buttons[15] = 1 ;
            }
            else if ( key == KEY_H || key == KEY_h ) {
                buttons[16] = 1 ;
            }
            else if ( key == KEY_M || key == KEY_m ) {
                buttons[17] = 1 ;
            }
            else if ( key == KEY_N || key == KEY_n ) {
                buttons[18] = 1 ;
            }
            else {
                ROS_INFO( "/keyboard: keycode is: %d. Ignored Key.", key ) ;
                continue ;
            }

            // Publish buttons
            sensor_msgs::Joy msg ;
            msg.header.stamp = ros::Time::now() ;
            msg.header.frame_id = "keyboard" ;
            msg.buttons.resize(NBUTTONS) ;
            for( int iter = 0 ; iter != NBUTTONS ; iter++ ) {
                msg.buttons[iter] = buttons[iter] ;
            }
            pub.publish(msg) ;
            msg.header.stamp = ros::Time::now() ;
            msg.buttons.resize(NBUTTONS) ;
            for( int iter = 0 ; iter != NBUTTONS ; iter++ ) {
                msg.buttons[iter] = 0 ;
            }
            pub.publish(msg) ; // Everything to 0
            //ros::Duration(0.1).sleep();
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt) ;
        ros::shutdown() ;
        exit(0) ;
    }
} ;


// Stop handler changes flag
void keyboardStopHandler (int sig) {
    keyboard_requested_shutdown = 1 ;
}


int main(int argc, char **argv)
{
    ros::init( argc, argv, "keyboard" ) ;
    signal( SIGINT, keyboardStopHandler ) ;
    Keyboard keyboard ;
    keyboard.init() ;
    ros::spin() ;

    // Quit
    tcsetattr(STDIN_FILENO, TCSANOW, &keyboard.oldt) ;
    ros::shutdown() ;
    exit(0) ;
    return 0 ;
}
