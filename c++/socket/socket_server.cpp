#include<stdlib>
#include<stdio>
#include<sys/socket.h>
#include<stdint>
#include<string.h>
#include<errno.h>
#include<unistd.h>
    
#define IP_ADDRESS "192.168.0.16"


bool ErrorCheck(const int ret_val, const char* p_error_step)
{
    if(-1 == ret_val)
    {
        printf("Error during %s: %i. Error descritption: %s", p_error_step, errno, strerror(errno));
        exit(errno); 
    }

    return true;
}

bool SocketIntialization(int *p_socket_fd)
{
    bool ret_val = false;

    if(ErrorCheck((socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0)), "socket initialization"))
    {
        struct socketaddr_in socket_address;
        socket_address.sin_family = AF_INET;
        socket_address.sin_port = 59000;
        socket_address.sin_addr.s_addr = inet_addr(IP_ADDRESS);
        
        if(ErrorCheck(bind(fd, (struct socketaddr *) &socket_address, sizeof(struct socketaddr_in)), "Binding socket"))
            ret_val = true;
    }

    return ret_val;
}


int main(void)
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    
    if(SocketInitialization(socket_fd))
    {
        
    }

    return 0;
}
