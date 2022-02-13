#include<stdlib.h>
#include<stdio.h>
#include<sys/socket.h>
#include<stdint.h>
#include<string.h>
#include<errno.h>
#include<unistd.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<sys/time.h>
#include<sys/select.h>


#define DATAGRAM_CURR_ADDRESS "192.168.0.16"
#define DATAGRAM_PEER_ADDRESS "192.168.0.16"
#define DATAGRAM_PORT 59001
#define DATAGRAM_PEER_PORT 59002
#define ONE_SECOND 1000

struct sockaddr_in g_curr_socket;
struct sockaddr_in g_peer_socket;
struct sockaddr_in g_peer_addr;

const uint32_t READ_SOCKET_BUFFER_SIZE = 100;
const uint32_t WRITE_SOCKET_BUFFER_SIZE = 100;

bool ErrorCheck(const int ret_val, const char* p_error_step)
{
    if(-1 == ret_val)
    {
        printf("Error during %s: %i. Error descritption: %s\n", p_error_step, errno, strerror(errno));
        exit(errno);
    }
    else
    {
        printf("%s: succedded\n", p_error_step);
    }

    return true;
}

bool SocketInitialization(int *p_socket_fd)
{
    bool ret_val = false;

    g_curr_socket = {.sin_family = AF_INET, .sin_port = htons(DATAGRAM_PORT)};
    g_peer_socket = {.sin_family = AF_INET, .sin_port = htons(DATAGRAM_PEER_PORT)};
    g_curr_socket.sin_addr.s_addr = inet_addr(DATAGRAM_CURR_ADDRESS);
    g_peer_socket.sin_addr.s_addr = inet_addr(DATAGRAM_PEER_ADDRESS);


    memset(&g_peer_addr, 0, sizeof(sockaddr_in));

    *p_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(ErrorCheck(*p_socket_fd, "socket initialization"))
    {
        if(ErrorCheck(bind(*p_socket_fd, (struct sockaddr *) &g_curr_socket, sizeof(g_curr_socket)), "Binding socket"))
            ret_val = true;
    }

    return ret_val;
}

void ReadSocketData(const int socket_fd)
{
    char data_from_peer[READ_SOCKET_BUFFER_SIZE] = {};
    int peer_size = sizeof(struct sockaddr_in);

    //peer size must be initialized with the appropriate struct size
    recvfrom(socket_fd, data_from_peer, READ_SOCKET_BUFFER_SIZE,
            0, (struct sockaddr *) &g_peer_addr, (socklen_t *) &peer_size);

    if(strlen(data_from_peer))
    {
        printf("Message received from %s. Data content: %s\n", inet_ntoa(g_peer_addr.sin_addr),
            data_from_peer);
    }
}

void WriteSocketData(const int socket_fd, const char *message)
{
    if(strlen(message) && g_peer_addr.sin_addr.s_addr)
    {
        sendto(socket_fd, (void *) message, strlen(message),
               MSG_ERRQUEUE, (struct sockaddr *) &g_peer_addr, sizeof(struct sockaddr_in));
    }
}

void reset_fd_sets(fd_set *fd_select, int fd)
{
    if(NULL != fd_select)
    {
        FD_ZERO(fd_select);
        FD_SET(fd, fd_select);
    }
}

int main(void)
{
    int socket_fd;
    char input_buffer[WRITE_SOCKET_BUFFER_SIZE] = {};

    if(SocketInitialization(&socket_fd))
    {
        fd_set read_fd; //It represents the read monitored file descriptors
        fd_set write_fd; //It represents the write monitored file descriptors

        struct timeval time_interval = {.tv_sec = 0, .tv_usec = 300000};

        for(;;)
        {
            int ret_val = 0;
            reset_fd_sets(&read_fd, socket_fd);
            reset_fd_sets(&write_fd, socket_fd);

            /*
            Select syscall alters the content of the fd_sets.
            Only the file descriptors with modifications remains in the sets.
            Each time a verification is done, the sets should be reconfigured
            */
            if(ret_val = select(socket_fd + 1, &read_fd, &write_fd, NULL, NULL))
            {
                if(FD_ISSET(socket_fd, &read_fd))
                    ReadSocketData(socket_fd);

                else if(FD_ISSET(socket_fd, &write_fd))
                    WriteSocketData(socket_fd, input_buffer);
            }
            else if(-1 == ret_val)
            {
                ErrorCheck(ret_val, "monitoring socket via poll");
            }
        }
    }

    return 0;
}
