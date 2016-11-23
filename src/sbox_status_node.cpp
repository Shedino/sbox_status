#include "ros/ros.h"

//#include <sherpa_msgs/Safety.h>// input
//#include <sherpa_msgs/LeashingStatus.h>// input
//#include "geometry_msgs/Vector3.h"// input
#include <sbox_status/Sbox_msg_status.h>
#include "sbox_status/Sbox_msg_commands.h"

//Serial includes
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class sbox_status_node {
	private:
	uint16_t counter_print;
	uint16_t seq_number;

	protected:
	char const *portname = "/dev/ttyUSB0";
	int fd;
	int wlen;
	/*state here*/
	ros::NodeHandle _n;

	// Subscribers
	//ros::Subscriber subFromCmd_;
	//sherpa_msgs::Cmd inputCmd_;
	// Publishers
	//ros::Publisher pubToAckMission_;
	//sherpa_msgs::Ack_mission outputAckMission_;

	int rate;


	int set_interface_attribs(int fd, int speed)
	{
	    struct termios tty;

	    if (tcgetattr(fd, &tty) < 0) {
	        printf("Error from tcgetattr: %s\n", strerror(errno));
	        return -1;
	    }

	    cfsetospeed(&tty, (speed_t)speed);
	    cfsetispeed(&tty, (speed_t)speed);

	    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	    tty.c_cflag &= ~CSIZE;
	    tty.c_cflag |= CS8;         /* 8-bit characters */
	    tty.c_cflag &= ~PARENB;     /* no parity bit */
	    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	    /* setup for non-canonical mode */
	    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	    tty.c_oflag &= ~OPOST;

	    /* fetch bytes as they become available */
	    tty.c_cc[VMIN] = 1;
	    tty.c_cc[VTIME] = 1;

	    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
	        printf("Error from tcsetattr: %s\n", strerror(errno));
	        return -1;
	    }
	    return 0;
	}

	void set_mincount(int fd, int mcount)
	{
	    struct termios tty;

	    if (tcgetattr(fd, &tty) < 0) {
	        printf("Error tcgetattr: %s\n", strerror(errno));
	        return;
	    }

	    tty.c_cc[VMIN] = mcount ? 1 : 0;
	    tty.c_cc[VTIME] = 5;        /* half second timer */

	    if (tcsetattr(fd, TCSANOW, &tty) < 0)
	        printf("Error tcsetattr: %s\n", strerror(errno));
	}


	
	public:
	sbox_status_node(ros::NodeHandle& node)
	{
		rate = 100;
		_n=node;

		//*portname = "/dev/ttyUSB0";

		//subscribers
		//subFromCmd_=n_.subscribe("/sent_command", 10, &MmsNodeClass::readCmdMessage,this);

		// publishers
		//pubToAckMission_=n_.advertise<sherpa_msgs::Ack_mission>("/ack_mission", 10);


		fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0) {
			printf("Error opening %s: %s\n", portname, strerror(errno));
			//return -1;
		}
		/*baudrate 115200, 8 bits, no parity, 1 stop bit */
		set_interface_attribs(fd, B115200);
		//set_mincount(fd, 0);                /* set to pure timed read */

		/*
		// simple output
		wlen = write(fd, "Hello!\n", 7);
		if (wlen != 7) {
			printf("Error from write: %d, %d\n", wlen, errno);
		}
		tcdrain(fd);    // delay for output


		do {
			unsigned char buf[80];
			int rdlen;

			rdlen = read(fd, buf, sizeof(buf) - 1);
			if (rdlen > 0) {
	#ifdef DISPLAY_STRING
				buf[rdlen] = 0;
				printf("Read %d: \"%s\"\n", rdlen, buf);
	#else // display hex
				unsigned char   *p;
				printf("Read %d:", rdlen);
				for (p = buf; rdlen-- > 0; p++)
					printf(" 0x%x", *p);
				printf("\n");
	#endif
			} else if (rdlen < 0) {
				printf("Error from read: %d: %s\n", rdlen, strerror(errno));
			}
			// repeat read to get full message
		} while (1);
		*/

	}

/*
 * write (fd, "hello!\n", 7);           // send 7 character greeting

		usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
		                                     // receive 25:  approx 100 uS per char transmit
		char buf [100];
		int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
 *
 */
		//void readCmdMessage(const sherpa_msgs::Cmd::ConstPtr& msg)
		//{
//
//		}

	void run()
	{
		ROS_INFO("SBOX_STATUS: RUNNING");
		ros::Rate loop_rate(rate);

		while (ros::ok())
		{
			ROS_INFO("SBOX_STATUS: STEP");

			ros::spinOnce();

			loop_rate.sleep();
		}
	}
	

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sbox_status_node");
	ros::NodeHandle node;

	sbox_status_node sboxstnode(node);
	sboxstnode.run();
	
	return 0;
}
