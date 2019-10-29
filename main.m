#include <errno.h>
#include <termios.h>
#include <unistd.h>
#import <Foundation/Foundation.h>
#import <sys/event.h>

int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        return -1;
    }
    
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag |= (IGNPAR);
    tty.c_iflag &= ~(IXON | IXOFF | INLCR | IGNCR);
    tty.c_oflag &= ~OPOST;
    
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_iflag &= ~INPCK;
    tty.c_cflag &= ~CSTOPB;
    tty.c_iflag |= INPCK;
    tty.c_cc[VTIME] = 1;  //  1s=10   0.1s=1 *
    tty.c_cc[VMIN] = 0;
    tty.c_lflag &= ~(ICANON|ECHO|ISIG|IEXTEN);
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        return -1;
    }
    return 0;
}

void
set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        return;
    }
    
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;
    
    tcsetattr (fd, TCSANOW, &tty);
    
}
void usage(char* n) {
    printf("usage: %s [/dev/cu.usbserial-XXXXXXX]\n", n);
    exit(-1);
}
int kq = 0;
int main(int argc, char * argv[]) {
    if(argc!=2) usage(argv[0]);
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (1, &tty) != 0)
    {
        return -1;
    }
    
    const char *portname = argv[1];
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0)
        usage(argv[0]);

    if (flock(fd, LOCK_EX|LOCK_NB) != 0) {
        printf("error: device %s is locked\n", portname);
        exit(-1);
    }
    
    assert(set_interface_attribs (fd, B115200, 0) == 0);
    assert(set_interface_attribs (1, B115200, 0) == 0);
    set_blocking (fd,0);
    struct kevent ke;
    kq = kqueue();
    assert(kq != -1);
    char buf[1024];
    EV_SET(&ke, fd, EVFILT_READ, EV_ADD, 0, 5, NULL);
    kevent(kq, &ke, 1, NULL, 0, NULL);
    EV_SET(&ke, 0, EVFILT_READ, EV_ADD, 0, 5, NULL);
    kevent(kq, &ke, 1, NULL, 0, NULL);
    printf("[connected, use CTRL+] to disconnect]\r\n");
    while (1) {
        memset(&ke, 0, sizeof(ke));
        int i = kevent(kq, NULL, 0, &ke, 1, NULL);
        if (i == 0)
            continue;
        if (ke.ident == fd) {
            int r = read(fd, buf, 1024);
            if (r > 0)
	            write(1, buf, r);
            else break;
        } else
            if (ke.ident == 0) {
                int r = read(0, buf, 1);
	        if (!(r > 0)) break;
                if (buf[0] == 0x1d) break;
                write(fd, buf, r);
            }
    }
    if (tcsetattr (1, TCSANOW, &tty) != 0)
    {
        return -1;
    }
    printf("\r\n[disconnected]\r\n");
    flock(fd, LOCK_UN);
    exit(0);
}

