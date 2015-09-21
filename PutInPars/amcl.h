#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define MAX_BUF 1024

template <typename T>
std::string to_string(T value) {
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}

class AMCL {
public:
	std::string amcl_map; // absolute directory

	AMCL(){}

	std::vector<double> AMCL_run(double x_tobe, double y_tobe, double angle_tobe) {
		
		std::vector<double> amcl_encode;
		std::string temp_str = "rosrun pars_ros amcl " + to_string(x_tobe) + ' ' + ' ' + to_string(y_tobe) + ' ' +  to_string(angle_tobe);

		pid_t pid = fork();

		if (pid > 0)
		{
			usleep(300000);
			std::cout<<"Sleep finished"<<std::endl;
			amcl_encode =  AMCL_print();
			return amcl_encode;
		}
		else if (pid == 0)
		{
			// child process, call execution function
			std::cerr<<"system call to be sent is "<<temp_str.c_str()<<std::endl;
    		system(temp_str.c_str());
    		exit(0); // end the copy of fork on this side
		}
		else
			std::cerr<<"Fail to fork new process"<<std::endl;
	}
	std::vector<double> AMCL_print()
	{
	    int fd;
	    const char * myfifo = "/tmp/myfifo";
	    double x, y, orientation_cos, orientation_sin;
	    double covariance[36];
	    std::vector<double> amcl_encapsulate;
	    /* open, read, and display the message from the FIFO */
	    fd = open(myfifo, O_RDONLY);
	    if (fd != -1)
	    {
		    //read(fd, buf, MAX_BUF);
		    read(fd, &x, sizeof(double));
		    read(fd, &y, sizeof(double));
		    read(fd, &orientation_cos, sizeof(double));
		    read(fd, &orientation_sin, sizeof(double));

		    for (int i=0; i<36; i++)
		    	read(fd, &covariance[i], sizeof(double));

		    close(fd);
		    std::cout<<"Read from PIPE (x, y): (" << x <<", "<< y <<")"<<std::endl;
		    std::cerr<<"Read from PIPE (x, y): (" << x <<", "<< y <<")"<<std::endl;

		    amcl_encapsulate.push_back(x);
		    amcl_encapsulate.push_back(y);
		    amcl_encapsulate.push_back(orientation_cos);
		    amcl_encapsulate.push_back(orientation_sin);
		    amcl_encapsulate.push_back(covariance[0]);//xx
		    amcl_encapsulate.push_back(covariance[1]);//xy
		    amcl_encapsulate.push_back(covariance[6]);//yx
		    amcl_encapsulate.push_back(covariance[7]);//yy
		    
		    return amcl_encapsulate;
		}
	}
};
