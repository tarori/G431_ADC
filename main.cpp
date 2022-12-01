#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Serial port headers
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#define DEV_NAME "/dev/ttyS4"
constexpr uint32_t baud_rate = 2000000;
constexpr uint32_t buff_size = 4096;

void serial_init(int fd)
{
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 100;
    cfsetispeed(&tio, baud_rate);
    cfsetospeed(&tio, baud_rate);
    tcsetattr(fd, TCSANOW, &tio);
}

void serial_write(int fd, char data)
{
    if (write(fd, &data, 1) != 1) {
        printf("Port write error\n");
    }
}

int main(int argc, char* argv[])
{
    int fd;
    fd = open(DEV_NAME, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s\n", DEV_NAME);
        return 1;
    }
    serial_init(fd);
    printf("Port opened\n");

    if (argc < 2) {
        printf("./run logname.txt\n");
        return 1;
    }

    int data_num = argc - 1;

    printf("Acquiring %d data\n", data_num);
    for (int i = 0; i < data_num; ++i) {
        FILE* fp = fopen(argv[i + 1], "w");
        serial_write(fd, 's');
        printf("Saving ");
        while (1) {
            char buf[buff_size];
            int size = read(fd, buf, sizeof(buf));
            int i = 0;
            for (i = 0; i < size; ++i) {
                // printf("%c", buf[i]);
                if (buf[i] == '\r') {
                    size = -1;
                    break;
                }
                if (buf[i] == '\0') {
                    printf("Hur?\n");
                    continue;
                }
                fputc(buf[i], fp);
            }
            printf(".");
            fflush(stdout);
            if (size < 0) {
                break;
            }
        }
        printf("\n%d Data end\n", i + 1);
        fclose(fp);
        sleep(1);
    }
    return 0;
}
